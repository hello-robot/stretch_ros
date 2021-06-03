from numba import jit, njit

import numpy as np

@njit(fastmath=True)
def numba_sample_ridge(window_width, ridge_mask, distance_map, distance_threshold):
    # Currently, this does not properly handle the borders of the
    # image (i.e., <= window_width of the image edges.
    h, w = ridge_mask.shape

    sample_map = np.zeros_like(ridge_mask, np.uint8)

    l_y_max = h - window_width
    l_x_max = w - window_width
    l_y = 0
    while l_y < l_y_max:
        l_x = 0
        while l_x < l_x_max:
            max_found = False
            max_value = 0
            max_y = 0
            max_x = 0
            w_y = 0
            while w_y < window_width:
                y = l_y + w_y
                w_x = 0
                while w_x < window_width: 
                    x = l_x + w_x
                    if ridge_mask[y,x] > 0:
                        value = distance_map[y,x]
                        if (value > distance_threshold) and (value > max_value):
                            max_found = True
                            max_value = value
                            max_y = y
                            max_x = x
                    w_x += 1
                w_y += 1
            if max_found:
                sample_map[max_y, max_x] = 255
            l_x += window_width
        l_y += window_width

    l_y = window_width//2
    while l_y < l_y_max:
        l_x = window_width//2
        while l_x < l_x_max:
            max_found = False
            max_value = 0
            max_y = 0
            max_x = 0
            w_y = 0
            while w_y < window_width:
                y = l_y + w_y
                w_x = 0
                while w_x < window_width: 
                    x = l_x + w_x
                    if sample_map[y,x] > 0:
                        sample_map[y,x] = 0
                        value = distance_map[y,x]
                        if value > max_value:
                            max_found = True
                            max_value = value
                            max_y = y
                            max_x = x
                    w_x += 1
                w_y += 1
            if max_found:
                sample_map[max_y, max_x] = 255
            l_x += window_width
        l_y += window_width

    return sample_map



@njit(fastmath=True)
def numba_sample_ridge_list(window_width, ridge_mask, distance_map, distance_threshold):
    # Currently, this does not properly handle the borders of the
    # image (i.e., <= window_width of the image edges.
    h, w = ridge_mask.shape
    
    sample_map = np.zeros_like(ridge_mask, np.uint8)

    l_y_max = h - window_width
    l_x_max = w - window_width
    l_y = 0
    while l_y < l_y_max:
        l_x = 0
        while l_x < l_x_max:
            max_found = False
            max_value = 0
            max_y = 0
            max_x = 0
            w_y = 0
            while w_y < window_width:
                y = l_y + w_y
                w_x = 0
                while w_x < window_width: 
                    x = l_x + w_x
                    if ridge_mask[y,x] > 0:
                        value = distance_map[y,x]
                        if (value > distance_threshold) and (value > max_value):
                            max_found = True
                            max_value = value
                            max_y = y
                            max_x = x
                    w_x += 1
                w_y += 1
            if max_found:
                sample_map[max_y, max_x] = 255
            l_x += window_width
        l_y += window_width

    sample_list = []
    
    l_y = window_width//2
    while l_y < l_y_max:
        l_x = window_width//2
        while l_x < l_x_max:
            max_found = False
            max_value = 0
            max_y = 0
            max_x = 0
            w_y = 0
            while w_y < window_width:
                y = l_y + w_y
                w_x = 0
                while w_x < window_width: 
                    x = l_x + w_x
                    if sample_map[y,x] > 0:
                        value = distance_map[y,x]
                        if value > max_value:
                            max_found = True
                            max_value = value
                            max_y = y
                            max_x = x
                    w_x += 1
                w_y += 1
            if max_found:
                sample_list.append([max_x, max_y])
            l_x += window_width
        l_y += window_width

    return sample_list
