from numba import jit, njit


@njit(fastmath=True)
def numba_compare_images_2(image_to_warp, target_image, target_image_not_smoothed, affine_matrix, m_per_height_unit, match_threshold_m=0.07, weight_by_height=True):
    target_image_not_smoothed_height, target_image_not_smoothed_width = target_image_not_smoothed.shape
    target_image_height, target_image_width = target_image.shape
    image_to_warp_height, image_to_warp_width = image_to_warp.shape
    
    if (target_image_height != target_image_not_smoothed_height) or (target_image_width != target_image_not_smoothed_width):
        print('ERROR: numba_compare_images_2: target_image and target_image_not_smoothed must have the same dimensions, yet')
        print('       target_image_not_smoothed_width, target_image_not_smoothed_height =', (target_image_not_smoothed_width, target_image_not_smoothed_height))
        print('       target_image_width, target_image_height =', (target_image_width, target_image_height))
  
    threshold_unit = match_threshold_m / m_per_height_unit
    
    dx0 = affine_matrix[0,0]
    dx1 = affine_matrix[1,0]
    
    dy0 = affine_matrix[0,1]
    dy1 = affine_matrix[1,1]
    
    b0 = affine_matrix[0,2]
    b1 = affine_matrix[1,2]

    match_score = 0.0
    
    # 0.5 shift to compensate for casting to int instead of rounding
    start_0 = (b0 - dy0) + 0.5 
    start_1 = (b1 - dy1) + 0.5 
    for to_warp_y in range(image_to_warp_height):
        start_0 = start_0 + dy0
        start_1 = start_1 + dy1
        target_x_f = start_0 - dx0
        target_y_f = start_1 - dx1
        for to_warp_x in range(image_to_warp_width):
            target_x_f = target_x_f + dx0
            target_y_f = target_y_f + dx1
            to_warp_value = image_to_warp[to_warp_y, to_warp_x]
            if to_warp_value != 0: 
                target_x = int(target_x_f)
                target_y = int(target_y_f)
                if (target_x >= 0) and (target_x < target_image_width) and (target_y >= 0) and (target_y < target_image_height):
                    target_not_smoothed_value = target_image_not_smoothed[target_y, target_x]
                    if target_not_smoothed_value != 0:
                        target_value = target_image[target_y, target_x]
                        difference = abs(target_value - to_warp_value)
                        if difference < threshold_unit:
                            if weight_by_height:
                                match_score = match_score + target_value
                            else:
                                match_score = match_score + 1.0
    return match_score

