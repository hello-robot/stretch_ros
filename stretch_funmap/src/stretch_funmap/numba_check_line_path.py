from numba import jit, njit
import numpy as np

@njit(fastmath=True)
def in_bounds(xy, h, w):
    x, y = xy
    return ((x >= 0) and (x < w) and (y >= 0) and (y < h))

@njit(fastmath=True)
def numba_check_line_path(start_xy, end_xy, distance_map, distance_threshold):
    h, w = distance_map.shape

    if not in_bounds(start_xy, h, w):
        return None

    if not in_bounds(end_xy, h, w):
        return None
    
    diff_xy = end_xy - start_xy
    length = np.sqrt( (diff_xy[0] * diff_xy[0]) + (diff_xy[1] * diff_xy[1]) )
    delta_xy = diff_xy / length
    dx = 0.5 * delta_xy[0]
    dy = 0.5 * delta_xy[1]

    x, y = start_xy
    end_x, end_y = end_xy
    
    while (x <= end_x) and (y <= end_y):
        x_i = int(round(x))
        y_i = int(round(y))
        val = distance_map[y_i, x_i]
        if val < distance_threshold:
            return False
        x += dx
        y += dy

    return True


@njit(fastmath=True)
def numba_find_contact_along_line_path(start_xy, end_xy, mask_image):
    h, w = mask_image.shape

    if not in_bounds(start_xy, h, w):
        return None, (None, None)

    if not in_bounds(end_xy, h, w):
        return None, (None, None)
    
    diff_xy = end_xy - start_xy
    length = np.sqrt( (diff_xy[0] * diff_xy[0]) + (diff_xy[1] * diff_xy[1]) )
    delta_xy = diff_xy / length
    dx = 0.5 * delta_xy[0]
    dy = 0.5 * delta_xy[1]

    x, y = start_xy
    end_x, end_y = end_xy
    x_i = int(round(x))
    y_i = int(round(y))
    end_x_i = int(round(end_x))
    end_y_i = int(round(end_y))

    while not ((x_i == end_x_i) and (y_i == end_y_i)):  
        x_i = int(round(x))
        y_i = int(round(y))
        val = mask_image[y_i, x_i]
        if val > 0:
            # contact_found, (contact_x, contact_y)
            return True, (x_i, y_i)
        x += dx
        y += dy

    # contact_found, (contact_x, contact_y)
    return False, (None, None)


@njit(fastmath=True)
def numba_find_line_path_on_surface(start_xy, end_xy, surface_mask_image, obstacle_mask_image):
    # Find where a linear path overlaps a surface without contacting an obstacle.
    s_h, s_w = surface_mask_image.shape
    o_h, o_w = obstacle_mask_image.shape

    if (s_h != o_h) or (s_w != o_w):
        return None, None, None

    h, w = s_h, s_w
    
    if not in_bounds(start_xy, h, w):
        return None, None, None

    if not in_bounds(end_xy, h, w):
        return None, None, None

    # prepare to move along the line
    diff_xy = end_xy - start_xy
    length = np.sqrt( (diff_xy[0] * diff_xy[0]) + (diff_xy[1] * diff_xy[1]) )
    delta_xy = diff_xy / length
    dx = 0.5 * delta_xy[0]
    dy = 0.5 * delta_xy[1]

    x, y = start_xy
    end_x, end_y = end_xy
    x_i = int(round(x))
    y_i = int(round(y))
    end_x_i = int(round(end_x))
    end_y_i = int(round(end_y))

    first_surface_contact_xy = None
    last_surface_contact_xy = None
    first_obstacle_contact_xy = None
    
    while not ((x_i == end_x_i) and (y_i == end_y_i)):  
        x_i = int(round(x))
        y_i = int(round(y))
        if surface_mask_image[y_i, x_i] > 0:
            # surface overlap
            if first_surface_contact_xy is None:
                first_surface_contact_xy = [x_i, y_i]
            last_surface_contact_xy = [x_i, y_i]
        if obstacle_mask_image[y_i, x_i] > 0:
            first_obstacle_contact_xy = [x_i, y_i]
            return first_surface_contact_xy, last_surface_contact_xy, first_obstacle_contact_xy
        x += dx
        y += dy
    return first_surface_contact_xy, last_surface_contact_xy, first_obstacle_contact_xy
    
