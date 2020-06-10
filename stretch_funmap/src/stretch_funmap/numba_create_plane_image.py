from numba import njit
import numpy as np

@njit(fastmath=True)
def numba_create_plane_image(plane_parameters, image):

    image_height, image_width = image.shape
    alpha, beta, gamma = plane_parameters

    for y in range(image_height):
        for x in range(image_width):
            image[y, x] = (alpha * x) + (beta * y) + gamma

            
@njit(fastmath=True)
def transform_original_to_corrected(plane_parameters, new_plane_height):
    # This does not account for clipping    
    alpha, beta, gamma = plane_parameters
    
    transform = np.array([[1.0,      0.0, 0.0, 0.0],
                          [0.0,      1.0, 0.0, 0.0],
                          [-alpha, -beta, 1.0, new_plane_height - gamma],
                          [0.0,      0.0, 0.0, 1.0]])
    return transform


@njit(fastmath=True)
def transform_corrected_to_original(plane_parameters, new_plane_height):
    transform = transform_original_to_corrected(plane_parameters, new_plane_height)
    return np.linalg.inv(transform)

@njit(fastmath=True)
def numba_correct_height_image(plane_parameters, height_image, new_plane_height):
    # Sets the plane described by the plane parameters to have a
    # constant height equal to plane_height. This does not perform a
    # true rotation and instead subtracts the plane from the current
    # heights. This is a good approximation for small angles, which
    # corresponds with planes that are close to horizontal (i.e.,
    # close to constant height).

    # Currently, this assumes that the height image has type uint8
    
    new_height_image = np.zeros_like(height_image)
    
    image_height, image_width = height_image.shape
    alpha, beta, gamma = plane_parameters

    for y in range(image_height):
        for x in range(image_width):
            height = height_image[y,x]
            if height != 0:
                plane_z = (alpha * x) + (beta * y) + gamma
                new_height = (height - plane_z) + new_plane_height
                new_height = np.round(new_height)

                # clip the height
                if new_height < 1:
                    # If the max height is too low, set the pixel to
                    # being unobserved, since the point would not have
                    # been observed.
                    new_height = 0
                if new_height > 255:
                    # If the max height is too high, set the pixel to
                    # the maximum height possible. This assumes that
                    # it's likely that the maximum height is not an
                    # isolated point at the given height. It can also
                    # be considered a conservative choice in terms of
                    # possible obstacles.
                    new_height = 255

                # This should cast the value to a uint8
                new_height_image[y,x] = new_height

    transform = transform_original_to_corrected(plane_parameters, new_plane_height)
    
    return new_height_image, transform
