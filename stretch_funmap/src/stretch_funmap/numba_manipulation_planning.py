from numba import jit, njit
import numpy as np

@njit(fastmath=True)
def numba_find_base_poses_that_reach_target(target_x, target_y, num_angles, pix_per_m,
                                            start_distance_m, max_arm_travel_m,
                                            origin_offset_left_m, origin_offset_length_m,
                                            dilated_obstacle_image): 
    h, w = dilated_obstacle_image.shape

    base_xy_image = np.zeros_like(dilated_obstacle_image)
    base_ang_image = np.zeros((h,w), np.float32)
    arm_reach_image = np.zeros((h,w), np.float32)
    
    start_distance = pix_per_m * start_distance_m
    # maximum travel of the telescoping arm in meters
    max_length = (pix_per_m * max_arm_travel_m) + start_distance

    origin_offset_left = pix_per_m * origin_offset_left_m
    origin_offset_length = pix_per_m * origin_offset_length_m
    
    delta_base_ang = (2.0 * np.pi) / num_angles
    
    base_ang = 0.0
    
    for n in range(num_angles):
        base_ang = base_ang + delta_base_ang
        arm_ang = base_ang - (np.pi/2.0)
        line_ang = arm_ang + np.pi
        dx = np.cos(line_ang)
        dy = -np.sin(line_ang)

        base_offset_x = (origin_offset_length * dx) + (origin_offset_left * -dy)
        base_offset_y = (origin_offset_length * dy) + (origin_offset_left *  dx)
        
        dx_total = dx
        dy_total = dy
        delta_length = np.sqrt((dx*dx) + (dy*dy))
        length = delta_length
        x_float = target_x + dx_total
        y_float = target_y + dy_total
        x = int(round(x_float))
        y = int(round(y_float))
        
        collision = False
        
        while ((not collision) and
               (length < max_length) and
               (x < w) and (y < h) and
               (x >= 0) and (y >= 0)):
            # Check if the straightline reaching model would collide
            # with an object at the point along the ray extending from
            # the target.
            if dilated_obstacle_image[y,x] == 0:
                if length > start_distance:
                    # The length of the gripper has been exceeded, so
                    # valid base poses begin and should be recorded.
                    base_x = int(round(x_float + base_offset_x))
                    base_y = int(round(y_float + base_offset_y))
                    base_xy_image[base_y, base_x] = 255
                    base_ang_image[base_y, base_x] = base_ang
                    arm_reach_image[base_y, base_x] = length - start_distance
            else:
                collision = True
            dx_total += dx
            dy_total += dy
            length += delta_length
            x_float = target_x + dx_total
            y_float = target_y + dy_total
            x = int(round(x_float))
            y = int(round(y_float))
            
    return base_xy_image, base_ang_image, arm_reach_image


@njit(fastmath=True)
def numba_check_that_tool_can_deploy(base_xy_image, base_ang_image, dilated_obstacle_image,
                                     origin_to_yaw_left_m, origin_to_yaw_length_m, pix_per_m):
    h, w = dilated_obstacle_image.shape

    new_base_xy_image = np.zeros_like(dilated_obstacle_image)

    origin_to_yaw_left_pix = origin_to_yaw_left_m * pix_per_m
    origin_to_yaw_length_pix = origin_to_yaw_length_m * pix_per_m

    for y in range(h):
        for x in range(w):
            if base_xy_image[y,x] > 0:
                # Test this base pose for deployment collision.
                base_ang = base_ang_image[y,x]
                arm_ang = base_ang - (np.pi/2.0)
                c = np.cos(arm_ang)
                s = -np.sin(arm_ang)
                yaw_x = x + (c * origin_to_yaw_length_pix) + (s * origin_to_yaw_left_pix) 
                yaw_y = y + (s * origin_to_yaw_length_pix) + (-c * origin_to_yaw_left_pix)
                obstacle_x = int(round(yaw_x))
                obstacle_y = int(round(yaw_y))
                if dilated_obstacle_image[obstacle_y, obstacle_x] == 0:
                    new_base_xy_image[y,x] = 255

    return new_base_xy_image
