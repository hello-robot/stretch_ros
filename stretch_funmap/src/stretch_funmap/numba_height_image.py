
from numba import jit, njit
import numpy as np
import math


def numba_max_height_image_to_points(points_in_image_to_frame_mat, image, points, m_per_pix, m_per_height_unit):
    dtype = image.dtype
    
    if np.issubdtype(dtype, np.integer):
        return numba_max_height_image_to_points_int(points_in_image_to_frame_mat, image, points, m_per_pix, m_per_height_unit)
    elif np.issubdtype(dtype, np.floating):
        return numba_max_height_image_to_points_float(points_in_image_to_frame_mat, image, points, m_per_pix, m_per_height_unit)


@njit(fastmath=True)
def numba_max_height_image_to_points_int(points_in_image_to_frame_mat, image, points, m_per_pix, m_per_height_unit):

    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = image.shape

    # Unpack the affine transformation matrix that transforms points
    # from the image's coordinate system to a target frame.
    r00, r01, r02, t0 = points_in_image_to_frame_mat[0]
    r10, r11, r12, t1 = points_in_image_to_frame_mat[1]
    r20, r21, r22, t2 = points_in_image_to_frame_mat[2]

    x_array = points['x']
    y_array = points['y']
    z_array = points['z']
    
    i = 0
    for y_i in range(im_height):
        for x_i in range(im_width):
            val = image[y_i, x_i]
            # check if observed
            if val != 0:
                # observed, so create a point
                z_i = val - 1

                x_m = x_i * m_per_pix
                y_m = y_i * -m_per_pix
                z_m = z_i * m_per_height_unit
                
                x_f = (r00 * x_m) + (r01 * y_m) + (r02 * z_m) + t0
                y_f = (r10 * x_m) + (r11 * y_m) + (r12 * z_m) + t1
                z_f = (r20 * x_m) + (r21 * y_m) + (r22 * z_m) + t2
                
                x_array[i] = x_f
                y_array[i] = y_f
                z_array[i] = z_f
                i += 1
    return i



@njit(fastmath=True)
def numba_max_height_image_to_points_float(points_in_image_to_frame_mat, image, points, m_per_pix, m_per_height_unit):

    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = image.shape

    # Unpack the affine transformation matrix that transforms points
    # from the image's coordinate system to a target frame.
    r00, r01, r02, t0 = points_in_image_to_frame_mat[0]
    r10, r11, r12, t1 = points_in_image_to_frame_mat[1]
    r20, r21, r22, t2 = points_in_image_to_frame_mat[2]

    x_array = points['x']
    y_array = points['y']
    z_array = points['z']
    
    i = 0
    for y_i in range(im_height):
        for x_i in range(im_width):
            z = image[y_i, x_i]
            # check if observed
            if z > 0.0:
                x_m = x_i * m_per_pix
                y_m = y_i * -m_per_pix
                
                x_f = (r00 * x_m) + (r01 * y_m) + (r02 * z) + t0
                y_f = (r10 * x_m) + (r11 * y_m) + (r12 * z) + t1
                z_f = (r20 * x_m) + (r21 * y_m) + (r22 * z) + t2
                
                x_array[i] = x_f
                y_array[i] = y_f
                z_array[i] = z_f
                i += 1
    return i


def numba_max_height_image(points_to_image_mat, points,
                           image, m_per_pix, m_per_height_unit,
                           voi_x_m, voi_y_m, voi_z_m, verbose=False):

    dtype = image.dtype
    
    if np.issubdtype(dtype, np.integer):
        bounds = calculate_voi_bounds( m_per_pix, m_per_height_unit, voi_x_m, voi_y_m, voi_z_m )
        numba_max_height_image_int_check(points_to_image_mat, 
                                         image, m_per_pix, m_per_height_unit,
                                         voi_x_m, voi_y_m, voi_z_m, bounds, verbose)
        numba_max_height_image_int(points_to_image_mat, points,
                               image, m_per_pix, m_per_height_unit,
                                   voi_x_m, voi_y_m, voi_z_m, bounds)
    elif np.issubdtype(dtype, np.floating):
        bounds = calculate_voi_bounds( m_per_pix, m_per_height_unit, voi_x_m, voi_y_m, voi_z_m )
        numba_max_height_image_float_check(points_to_image_mat,
                                           image, m_per_pix, m_per_height_unit,
                                           voi_x_m, voi_y_m, voi_z_m, bounds, verbose)
        numba_max_height_image_float(points_to_image_mat, points,
                                   image, m_per_pix, m_per_height_unit,
                                   voi_x_m, voi_y_m, voi_z_m, bounds)


def numba_max_height_and_rgb_images(points_to_image_mat, rgb_points,
                                    height_image, rgb_image,
                                    m_per_pix, m_per_height_unit,
                                    voi_x_m, voi_y_m, voi_z_m,
                                    verbose=False):

    dtype = height_image.dtype
    
    if np.issubdtype(dtype, np.integer):
        bounds = calculate_voi_bounds( m_per_pix, m_per_height_unit, voi_x_m, voi_y_m, voi_z_m )
        numba_max_height_image_int_check(points_to_image_mat,
                                         height_image, m_per_pix,
                                         m_per_height_unit, voi_x_m,
                                         voi_y_m, voi_z_m, bounds,
                                         verbose)
        assert(height_image.shape[:2] == rgb_image.shape[:2])
        numba_max_height_and_rgb_images_int(points_to_image_mat, rgb_points,
                                            height_image, rgb_image, m_per_pix,
                                            m_per_height_unit, voi_x_m,
                                            voi_y_m, voi_z_m, bounds)
    elif np.issubdtype(dtype, np.floating):
        print('ERROR: numba_max_height_and_rgb_images currently does not support float max images.')
        assert(False)

        
def numba_max_height_and_rgb_and_camera_depth_images(points_to_image_mat, rgb_points,
                                                     height_image, rgb_image, camera_depth_image,
                                                     m_per_pix, m_per_height_unit,
                                                     voi_x_m, voi_y_m, voi_z_m,
                                                     verbose=False):

    dtype = height_image.dtype
    
    if np.issubdtype(dtype, np.integer):
        bounds = calculate_voi_bounds( m_per_pix, m_per_height_unit, voi_x_m, voi_y_m, voi_z_m )
        numba_max_height_image_int_check(points_to_image_mat,
                                         height_image, m_per_pix,
                                         m_per_height_unit, voi_x_m,
                                         voi_y_m, voi_z_m, bounds,
                                         verbose)
        assert(height_image.shape[:2] == rgb_image.shape[:2])
        numba_max_height_and_rgb_and_camera_depth_images_int(points_to_image_mat, rgb_points,
                                                             height_image, rgb_image, camera_depth_image,
                                                             m_per_pix, m_per_height_unit,
                                                             voi_x_m, voi_y_m, voi_z_m, bounds)
    elif np.issubdtype(dtype, np.floating):
        print('ERROR: numba_max_height_and_rgb_images currently does not support float max images.')
        assert(False)
        
    
def calculate_voi_bounds( m_per_pix, m_per_height_unit, voi_x_m, voi_y_m, voi_z_m):
    # Calculate the borders of the volume of interest (VOI) in order
    # to only consider 3D points that are strictly within the VOI,
    # excluding the VOI's borders. This uses a 1000th of a millimeter
    # safety margin to exclude points almost on the VOI's border. This
    # ensures that the bounds on the image indices and the pixel type
    # are not violated.
    half_pix = m_per_pix / 2.0
    mm = 0.001
    safety_margin = mm/1000.0
    min_x = (- half_pix) + safety_margin
    max_x = (voi_x_m - half_pix) - safety_margin
    min_y = (half_pix - voi_y_m) + safety_margin
    max_y = half_pix - safety_margin
    min_z = 0.0 + safety_margin
    max_z = voi_z_m - safety_margin
    
    return np.array([min_x, max_x, min_y, max_y, min_z, max_z])


def numba_max_height_image_int_check(points_to_image_mat,
                                     image, m_per_pix, m_per_height_unit,
                                     voi_x_m, voi_y_m, voi_z_m, bounds, verbose=False):
    # Use this to check bounds and other properties prior to running
    # the real numba version, which does not perform safety checking.
    
    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = image.shape
    dtype = image.dtype

    # This function is only for integer pixel types.
    assert(np.issubdtype(dtype, np.integer))

    max_z_val = np.iinfo(dtype).max

    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    
    # Check that the image indices and image values will all be within bounds.
    min_x_index = int(round( min_x / m_per_pix ))
    max_x_index = int(round( max_x / m_per_pix ))
    min_y_index = - int(round( max_y / m_per_pix ))
    max_y_index = - int(round( min_y / m_per_pix ))
    min_z_index = 1 + int(round( min_z / m_per_height_unit ))
    max_z_index = 1 + int(round( max_z / m_per_height_unit ))
    
    if verbose: 
        print('image.shape =', image.shape)
        print('np.iinfo(image.dtype).max =', np.iinfo(image.dtype).max)
        print('min_x_index =', min_x_index)
        print('max_x_index =', max_x_index)
        print('min_y_index =', min_y_index)
        print('max_y_index =', max_y_index)
        print('min_z_index =', min_z_index)
        print('max_z_index =', max_z_index)
    
    assert(min_x_index == 0)
    assert(max_x_index == (im_width - 1))
    assert(min_y_index == 0)
    assert(max_y_index == (im_height - 1))
    assert(min_z_index == 1)
    assert(max_z_index <= max_z_val)


def numba_max_height_image_float_check(points_to_image_mat,
                                       image, m_per_pix, m_per_height_unit,
                                       voi_x_m, voi_y_m, voi_z_m, bounds, verbose=False):
    # Use this to check bounds and other properties prior to running
    # the real numba version, which does not perform safety checking.
    
    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = image.shape
    dtype = image.dtype

    # This function is only for integer pixel types.
    assert(np.issubdtype(dtype, np.floating))

    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    
    # Check that the image indices and image values will all be within bounds.
    min_x_index = int(round( min_x / m_per_pix ))
    max_x_index = int(round( max_x / m_per_pix ))
    min_y_index = - int(round( max_y / m_per_pix ))
    max_y_index = - int(round( min_y / m_per_pix ))
    
    if verbose: 
        print('image.shape =', image.shape)
        print('np.iinfo(image.dtype).max =', np.iinfo(image.dtype).max)
        print('min_x_index =', min_x_index)
        print('max_x_index =', max_x_index)
        print('min_y_index =', min_y_index)
        print('max_y_index =', max_y_index)
        print('min_z =', min_z)
    
    assert(min_x_index == 0)
    assert(max_x_index == (im_width - 1))
    assert(min_y_index == 0)
    assert(max_y_index == (im_height - 1))
    assert(min_z > 0.0)

             
@njit(fastmath=True)
def numba_max_height_image_int(points_to_image_mat, points,
                               image, m_per_pix, m_per_height_unit,
                               voi_x_m, voi_y_m, voi_z_m, bounds):
    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = image.shape

    # Unpack the affine transformation matrix that transforms points
    # into the image's coordinate system.
    r00, r01, r02, t0 = points_to_image_mat[0]
    r10, r11, r12, t1 = points_to_image_mat[1]
    r20, r21, r22, t2 = points_to_image_mat[2]

    min_x, max_x, min_y, max_y, min_z, max_z = bounds 
    
    n_points = points.shape[0]    
    for p in range(n_points):
        x_p = points[p,0]
        y_p = points[p,1]
        z_p = points[p,2]

        x = (r00 * x_p) + (r01 * y_p) + (r02 * z_p) + t0
        y = (r10 * x_p) + (r11 * y_p) + (r12 * z_p) + t1
        z = (r20 * x_p) + (r21 * y_p) + (r22 * z_p) + t2

        # Ensures that points are strictly within the volume of
        # interest (VOI) by comparing float representations. Consider
        # removing these checks and replacing them with integer
        # comparisons to ensure valid image indices and values. This
        # would require changing the definition of a MaxHeightImage or
        # doing something clever to handle the exclusion of points on
        # the borders of the VOI.
        if (x > min_x) and (x < max_x) and (y > min_y) and (y < max_y) and (z > min_z) and (z < max_z): 
            x_index = int(round( x / m_per_pix ))
            y_index = - int(round( y / m_per_pix ))
            # A value of 0 represents no observations, so add 1.
            z_val = 1 + int(round( z / m_per_height_unit ))
            current_z_val = image[y_index, x_index]
            if z_val > current_z_val:
                image[y_index, x_index] = z_val

                
@njit(fastmath=True)
def numba_max_height_image_int_2(points_to_image_mat, points,
                                 image, m_per_pix, m_per_height_unit,
                                 voi_x_m, voi_y_m, voi_z_m, max_quantized_height):
    # This version expects the transformation matrix
    # (points_to_image_mat) to handle the scaling and depth offset
    # required for conversion to the max height image. In other words,
    # it handles all the linear operations, so only nonlinear
    # operations like rounding and casting remain.
    
    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = image.shape

    # Unpack the affine transformation matrix that transforms points
    # into the image.
    r00, r01, r02, t0 = points_to_image_mat[0]
    r10, r11, r12, t1 = points_to_image_mat[1]
    r20, r21, r22, t2 = points_to_image_mat[2]
    
    n_points = points.shape[0]    
    for p in range(n_points):
        x_p = points[p,0]
        y_p = points[p,1]
        z_p = points[p,2]

        x = (r00 * x_p) + (r01 * y_p) + (r02 * z_p) + t0
        x_index = int(round(x))
        if (x_index >= 0) and (x_index < im_width):
            y = (r10 * x_p) + (r11 * y_p) + (r12 * z_p) + t1
            y_index = int(round(y))
            if (y_index >= 0) and (y_index < im_height):
                height = (r20 * x_p) + (r21 * y_p) + (r22 * z_p) + t2
                height_quantized = int(round(height))
                if (height_quantized > 0) and (height_quantized <= max_quantized_height):
                    current_height_quantized = image[y_index, x_index]
                    if height_quantized > current_height_quantized:
                        image[y_index, x_index] = height_quantized

                
@njit(fastmath=True)
def numba_max_height_and_rgb_images_int(points_to_image_mat, rgb_points,
                                        height_image, rgb_image, m_per_pix,
                                        m_per_height_unit, voi_x_m,
                                        voi_y_m, voi_z_m, bounds):
    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = height_image.shape

    # Unpack the affine transformation matrix that transforms points
    # into the image's coordinate system.
    r00, r01, r02, t0 = points_to_image_mat[0]
    r10, r11, r12, t1 = points_to_image_mat[1]
    r20, r21, r22, t2 = points_to_image_mat[2]

    min_x, max_x, min_y, max_y, min_z, max_z = bounds 
    
    n_points = rgb_points.shape[0]    
    for i in range(n_points):
        p = rgb_points[i]
        x_p = p['x']
        y_p = p['y']
        z_p = p['z']
        
        x = (r00 * x_p) + (r01 * y_p) + (r02 * z_p) + t0
        y = (r10 * x_p) + (r11 * y_p) + (r12 * z_p) + t1
        z = (r20 * x_p) + (r21 * y_p) + (r22 * z_p) + t2

        # Ensures that points are strictly within the volume of
        # interest (VOI) by comparing float representations. Consider
        # removing these checks and replacing them with integer
        # comparisons to ensure valid image indices and values. This
        # would require changing the definition of a MaxHeightImage or
        # doing something clever to handle the exclusion of points on
        # the borders of the VOI.
        if (x > min_x) and (x < max_x) and (y > min_y) and (y < max_y) and (z > min_z) and (z < max_z): 
            x_index = int(round( x / m_per_pix ))
            y_index = - int(round( y / m_per_pix ))
            # A value of 0 represents no observations, so add 1.
            z_val = 1 + int(round( z / m_per_height_unit ))
            current_z_val = height_image[y_index, x_index]
            if z_val > current_z_val:
                height_image[y_index, x_index] = z_val
                r = p['r']
                g = p['g']
                b = p['b']
                rgb_sum = r + g + b
                # If [r, g, b] = [0, 0, 0], color information for the
                # point is assumed not to exist and the color of the
                # point is ignored by not updating the corresponding
                # pixel of the RGB image.

                # Example of use: For the Intel ROS Wrapper for the
                # D435i camera, point clouds produced with full depth
                # image mode (full sized frustum) include points
                # without color information (i.e., texture) due to the
                # narrower field of view of the D435i's RGB camera. In
                # these cases the points have [r,g,b] = [0,0,0].
                if rgb_sum != 0:
                    rgb_image[y_index, x_index] = [p['b'], p['g'], p['r']]


                    
@njit(fastmath=True)
def numba_max_height_and_rgb_and_camera_depth_images_int(points_to_image_mat, rgb_points,
                                                         height_image, rgb_image, camera_depth_image,
                                                         m_per_pix, m_per_height_unit,
                                                         voi_x_m, voi_y_m, voi_z_m, bounds):
    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = height_image.shape

    # Unpack the affine transformation matrix that transforms points
    # into the image's coordinate system.
    r00, r01, r02, t0 = points_to_image_mat[0]
    r10, r11, r12, t1 = points_to_image_mat[1]
    r20, r21, r22, t2 = points_to_image_mat[2]

    # Assume that the camera is located at 0.0, 0.0, 0.0 in the point
    # cloud coordinate system. Consequently, the center of the camera
    # in the height image can be found by transforming it using
    # points_to_image_mat, which results in the following x and y
    # coordinates.
    camera_x = t0
    camera_y = t1
    
    min_x, max_x, min_y, max_y, min_z, max_z = bounds 
    
    n_points = rgb_points.shape[0]    
    for i in range(n_points):
        p = rgb_points[i]
        x_p = p['x']
        y_p = p['y']
        z_p = p['z']
        
        x = (r00 * x_p) + (r01 * y_p) + (r02 * z_p) + t0
        y = (r10 * x_p) + (r11 * y_p) + (r12 * z_p) + t1
        z = (r20 * x_p) + (r21 * y_p) + (r22 * z_p) + t2

        # Ensures that points are strictly within the volume of
        # interest (VOI) by comparing float representations. Consider
        # removing these checks and replacing them with integer
        # comparisons to ensure valid image indices and values. This
        # would require changing the definition of a MaxHeightImage or
        # doing something clever to handle the exclusion of points on
        # the borders of the VOI.
        if (x > min_x) and (x < max_x) and (y > min_y) and (y < max_y) and (z > min_z) and (z < max_z): 
            x_index = int(round( x / m_per_pix ))
            y_index = - int(round( y / m_per_pix ))
            # A value of 0 represents no observations, so add 1.
            z_val = 1 + int(round( z / m_per_height_unit ))
            current_z_val = height_image[y_index, x_index]
            if z_val > current_z_val:
                # The height value is the maximum encountered at this
                # pixel, so update the pixel in all the images.
                height_image[y_index, x_index] = z_val
                
                # 4 cm per unit results in 10.16 meter max = 254*0.04.
                # The D435i is listed as having an approximately 10
                # meter maximum range.
                x_dist = x - camera_x
                y_dist = y - camera_y
                floor_distance = math.sqrt((x_dist*x_dist) + (y_dist*y_dist))
                camera_depth = 1 + int(round(floor_distance/0.04))
                if camera_depth > 255:
                    camera_depth = 255
                if camera_depth < 0:
                    camera_depth = 0
                camera_depth_image[y_index, x_index] = camera_depth

                r = p['r']
                g = p['g']
                b = p['b']
                rgb_sum = r + g + b
                # If [r, g, b] = [0, 0, 0], color information for the
                # point is assumed not to exist and the color of the
                # point is ignored by not updating the corresponding
                # pixel of the RGB image.

                # Example of use: For the Intel ROS Wrapper for the
                # D435i camera, point clouds produced with full depth
                # image mode (full sized frustum) include points
                # without color information (i.e., texture) due to the
                # narrower field of view of the D435i's RGB camera. In
                # these cases the points have [r,g,b] = [0,0,0].
                if rgb_sum != 0:
                    rgb_image[y_index, x_index] = [p['b'], p['g'], p['r']]
                
             
@njit(fastmath=True)
def numba_max_height_image_float(points_to_image_mat, points,
                                 image, m_per_pix, m_per_height_unit,
                                 voi_x_m, voi_y_m, voi_z_m, bounds):
    # Update the max height image to represent the provided 3D
    # points. This function is for images with integer pixels.
    im_height, im_width = image.shape

    # Unpack the affine transformation matrix that transforms points
    # into the image's coordinate system.
    r00, r01, r02, t0 = points_to_image_mat[0]
    r10, r11, r12, t1 = points_to_image_mat[1]
    r20, r21, r22, t2 = points_to_image_mat[2]

    min_x, max_x, min_y, max_y, min_z, max_z = bounds 
    
    n_points = points.shape[0]    
    for p in range(n_points):
        x_p = points[p,0]
        y_p = points[p,1]
        z_p = points[p,2]

        x = (r00 * x_p) + (r01 * y_p) + (r02 * z_p) + t0
        y = (r10 * x_p) + (r11 * y_p) + (r12 * z_p) + t1
        z = (r20 * x_p) + (r21 * y_p) + (r22 * z_p) + t2

        # Ensures that points are strictly within the volume of
        # interest (VOI) by comparing float representations. Consider
        # removing these checks and replacing them with integer
        # comparisons to ensure valid image indices and values. This
        # would require changing the definition of a MaxHeightImage or
        # doing something clever to handle the exclusion of points on
        # the borders of the VOI.
        if (x > min_x) and (x < max_x) and (y > min_y) and (y < max_y) and (z > min_z) and (z < max_z): 
            x_index = int(round( x / m_per_pix ))
            y_index = - int(round( y / m_per_pix ))
            current_z = image[y_index, x_index]
            if z > current_z:
                image[y_index, x_index] = z

                

@njit(fastmath=True)
def numba_create_segment_image_uint8(segments_image, val_image, val_to_segment_id):
    num_val_bins = val_to_segment_id.shape[0]
    if num_val_bins != 256:
        # ERROR!
        return None
    
    im_height, im_width = val_image.shape
    for y in range(im_height):
        for x in range(im_width):
            val = val_image[y, x]
            # check if observed
            if val != 0:
                # observed, so take action
                segments_image[y,x] = val_to_segment_id[val]
    return True
