#!/usr/bin/env python3

import numpy as np
import cv2

def fit_plane_to_height_image(height_image, mask):
    # Perform a least squares fit of a plane to the masked region of
    # the height_image. Find the 3 element vector a for the equation
    # aX ~= z where X[:,i] = [x_i, y_i, 1]^T, z[i] = z_i and a=[alpha,
    # beta, gamma] such that alpha*x + beta*y + gamma ~= z .
    z = height_image[mask > 0]
    nonzero = cv2.findNonZero(mask)
    perform_test = False
    if perform_test: 
        print('z.shape =', z.shape)
        print(z)
        for n in range(10): 
            test_x, test_y = nonzero[n][0]
            test_z = height_image[test_y, test_x]
            
            print('x, y, z, z_test =', test_x, test_y, test_z, z[n])
    num_points, s1, s2 = nonzero.shape
    nonzero = np.reshape(nonzero, (num_points, 2))
    X_T = np.append(nonzero, np.ones((num_points,1)), axis=1)
    a0 = np.matmul(z, X_T)
    A1 = np.matmul(X_T.transpose(), X_T)
    A1 = np.linalg.inv(A1)
    a = np.matmul(a0, A1)
    X = X_T.transpose()
    # aX ~= z
    return a, X, z

def fit_plane_to_height_image_error(a, X, z):
    # Calculate the fit error for the plane.
    z_fit = np.matmul(a, X)
    fit_error = z - z_fit
    return fit_error, z_fit


def svd_fit(points, verbose=False): 
    # calculate and subtract the mean
    center = np.mean(points, axis=0)

    if verbose:
        print( 'center =', center )

    # make the point distribution have zero mean
    points_zero_mean = points - center

    if verbose: 
        print( 'points_zero_mean[:5] =', points_zero_mean[:5] )
        print( 'points_zero_mean.shape =', points_zero_mean.shape )

    # find the covariance matrix, C, for the data
    C = np.cov(points_zero_mean.transpose())

    # find the SVD of the covariance matrix
    u, s, vh = np.linalg.svd(C)

    e0 = np.reshape(u[:, 0], (3,1))
    e1 = np.reshape(u[:, 1], (3,1))
    e2 = np.reshape(u[:, 2], (3,1))
    
    center = np.reshape(center, (3,1))
        
    return center, e0, e1, e2


class FitPlane():
    def __init__(self):
        self.d = None
        self.n = None
        # defines the direction from points to the camera
        self.towards_camera = np.reshape(np.array([0.0, 0.0, -1.0]), (3,1))

    def set_plane(self, n, d):
        self.n = n
        self.d = d
        self.update()
        
    def update(self):
        return

    def get_plane_normal(self):
        return -self.n

    def get_plane_coordinate_system(self):
        z_p = -self.n
        # two options to avoid selecting poor choice that is almost
        # parallel to z_p
        x_approx = np.reshape(np.array([1.0, 0.0, 0.0]), (3,1))
        x_approx_1 = x_approx - (np.matmul(z_p.transpose(), x_approx) * z_p)
        x_approx = np.reshape(np.array([0.0, 1.0, 0.0]), (3,1))
        x_approx_2 = x_approx - (np.matmul(z_p.transpose(), x_approx) * z_p)
        x_approx_1_mag = np.linalg.norm(x_approx_1)
        x_approx_2_mag = np.linalg.norm(x_approx_2)
        if x_approx_1_mag > x_approx_2_mag: 
            x_p = x_approx_1 / x_approx_1_mag
        else:
            x_p = x_approx_2 / x_approx_2_mag
        y_p = np.reshape(np.cross(z_p.flatten(), x_p.flatten()), (3,1))

        p_origin = self.d * self.n
        return x_p, y_p, z_p, p_origin
        
        
    def get_points_on_plane(self, plane_origin=None, side_length=1.0, sample_spacing=0.01):
        x_p, y_p, z_p, p_origin = self.get_plane_coordinate_system()
        h = side_length/2.0
        if plane_origin is None: 
            plane_list = [np.reshape((x_p * alpha) + (y_p * beta) + p_origin, (3,))
                          for alpha in np.arange(-h, h, sample_spacing)
                          for beta in np.arange(-h, h, sample_spacing)]
        else:
            plane_origin = np.reshape(plane_origin, (3, 1))
            plane_list = [np.reshape((x_p * alpha) + (y_p * beta) + plane_origin, (3,))
                          for alpha in np.arange(-h, h, sample_spacing)
                          for beta in np.arange(-h, h, sample_spacing)]

        plane_array = np.array(plane_list)
        return plane_array

        
    def abs_dist(self, points_array):
        out = np.abs(np.matmul(self.n.transpose(), points_array.transpose()) - self.d).flatten()
        return out
        
    def height(self, points_array):
        # positive is closer to the camera (e.g., above floor)
        # negative is farther from the camera (e.g., below floor)?
        out = - (np.matmul(self.n.transpose(), points_array.transpose()) - self.d).flatten()
        return out

    def get_points_nearby(self, points_array, dist_threshold_mm):
        # return points that are within a distance from the current plane
        if (self.n is not None) and (self.d is not None): 
            dist = np.abs(np.matmul(self.n.transpose(), points_array.transpose()) - self.d).flatten()
            # only points < dist_threshold meters away from the plane are
            # considered in the fit dist_threshold = 0.2 #1.0 #0.5 #0.2

            dist_threshold_m = dist_threshold_mm / 1000.0
            thresh_test = np.abs(dist) < dist_threshold_m
            points = points_array[thresh_test, :]
        else:
            points = points_array
        return points
        
    
    def fit_svd(self, points_array,
                dist_threshold_mm=200.0,
                prefilter_points=False,
                verbose=True):
        # relevant numpy documentation for SVD:
        #
        # "When a is a 2D array, it is factorized as u @ np.diag(s) @ vh"
        #
        #" The rows of vh are the eigenvectors of A^H A and the
        # columns of u are the eigenvectors of A A^H. In both cases
        # the corresponding (possibly non-zero) eigenvalues are given
        # by s**2. "

        if prefilter_points:
            # only fit to points near the current plane
            points = self.get_points_nearby(points_array, dist_threshold_mm)
        else:
            points = points_array

        center, e0, e1, e2 = svd_fit(points, verbose)
        
        # find the smallest eigenvector, which corresponds to the
        # normal of the plane
        n = e2
        
        # ensure that the direction of the normal matches our convention
        approximate_up = self.towards_camera
        if np.matmul(n.transpose(), approximate_up) > 0.0:
            n = -n
        if verbose: 
            print( 'SVD fit' ) 
            print( 'n =', n )
            print( 'np.linalg.norm(n) =', np.linalg.norm(n) )

        #center = np.reshape(center, (3,1))
        d = np.matmul(n.transpose(), center)
        if verbose: 
            print( 'd =', d )

        self.d = d
        self.n = n
        if verbose: 
            print( 'self.d =', self.d )
            print( 'self.n =', self.n )
        self.update()
        
         
    def fit_ransac(self, points_array,
                   dist_threshold=0.2,
                   ransac_inlier_threshold_m=0.04,
                   use_density_normalization=False,
                   number_of_iterations=100,
                   prefilter_points=False,
                   verbose=True):
        # Initial RANSAC algorithm based on pseudocode on Wikipedia
        # https://en.wikipedia.org/wiki/Random_sample_consensus

        if prefilter_points:
            # only fit to points near the current plane
            dist_threshold_mm = dist_threshold * 1000.0
            points = self.get_points_nearby(points_array, dist_threshold_mm)
        else:
            points = points_array
            
        num_points = points.shape[0]
        indices = np.arange(num_points)

        ransac_threshold_m = ransac_inlier_threshold_m

        min_num_inliers = 100

        approximate_up = self.towards_camera
        
        # should be well above the maximum achievable error, since
        # error is average distance in meters

        best_model_inlier_selector = None
        best_model_inlier_count = 0
        
        for i in range(number_of_iterations):
            if verbose:
                print( 'RANSAC iteration', i )
            candidate_inliers = points[np.random.choice(indices, 3), :]
            c0, c1, c2 = candidate_inliers
            # fit plane to candidate inliers
            n = np.cross(c1 - c0, c2 - c0)
            if np.dot(n, approximate_up) > 0.0:
                n = -n
            n = np.reshape(n / np.linalg.norm(n), (3,1))
            c0 = np.reshape(c0, (3,1))
            d = np.matmul(n.transpose(), c0)

            dist = np.abs(np.matmul(n.transpose(), points.transpose()) - d).flatten()
            select_model_inliers = dist < ransac_threshold_m
            if use_density_normalization:
                inliers = points[select_model_inliers]
                # square grid with this many bins to a side, small
                # values (e.g., 10 and 20) can result in the fit being
                # biased towards edges of the planar region
                num_bins = 100 # num_bins x num_bins = total bins
                density_image, mm_per_pix, x_indices, y_indices = create_density_image(inliers, self, image_width_pix=num_bins, view_width_m=5.0, return_indices=True)
                density_image = np.reciprocal(density_image, where=density_image!=0.0)
                number_model_inliers = np.int(np.round(np.sum(density_image[y_indices, x_indices])))
            else:
                number_model_inliers = np.count_nonzero(select_model_inliers)
            if number_model_inliers > min_num_inliers:
                if verbose:
                    print( 'model found with %d inliers' % number_model_inliers )
                if number_model_inliers > best_model_inlier_count:
                    if verbose:
                        print( 'model has more inliers than the previous best model, so updating' )
                    best_model_n = n
                    best_model_d = d
                    best_model_inlier_count = number_model_inliers
                    best_model_inlier_selector = select_model_inliers
                    best_model_inliers = None
                    best_model_error = None
                elif number_model_inliers == best_model_inlier_count:
                    if verbose:
                        print( 'model has the same number of inliers as the previous best model, so comparing' )
                    model_inliers = points[select_model_inliers]
                    # error is the average distance of points from the plane
                    # sum_i | n^T p_i - d |
                    # should be able to make this faster by selecting from the already computed distances
                    new_error = np.average(np.abs(np.matmul(n.transpose(), model_inliers.transpose()) - d))
                    if best_model_inliers is None: 
                        best_model_inliers = points[best_model_inlier_selector]
                    if best_model_error is None:
                        # should be able to make this faster by
                        # selecting from the already computed
                        # distances
                        best_model_error = np.average(np.abs(np.matmul(best_model_n.transpose(), best_model_inliers.transpose()) - best_model_d))
                    if new_error < best_model_error:
                        if verbose:
                            print( 'model has a lower error than the previous model, so updating' )
                        best_model_n = n
                        best_model_d = d
                        best_model_inlier_count = number_model_inliers
                        best_model_inlier_selector = select_model_inliers
                        best_model_inliers = model_inliers
                        best_model_error = new_error
        if best_model_inlier_count > 0:
            if verbose:
                print( 'RANSAC FINISHED' ) 
                print( 'new model found by RANSAC:' )
            self.d = best_model_d
            self.n = best_model_n
            if verbose:
                print( 'self.d =', self.d )
                print( 'self.n =', self.n )
            self.update()
        else:
            print( 'RANSAC FAILED TO FIND A MODEL' )
