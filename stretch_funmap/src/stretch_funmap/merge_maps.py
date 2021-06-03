#!/usr/bin/env python3

import stretch_funmap.max_height_image as mh
import numpy as np
import scipy.ndimage as nd
import scipy.signal as si
import cv2
import math
import hello_helpers.hello_misc as hm
import stretch_funmap.navigation_planning as na
import copy
import time


from scipy.optimize import minimize, minimize_scalar


from stretch_funmap.numba_compare_images import numba_compare_images_2
import stretch_funmap.mapping as ma

import tf_conversions

import cma


def affine_transform_2d_point(affine_matrix, point):
    affine_point = np.ones(3)
    affine_point[:2] = point
    affine_point = np.reshape(affine_point, (3,1))
    new_point = np.matmul(affine_matrix, affine_point)
    new_point = np.reshape(new_point, (2))
    return new_point


def register_images(max_height_image_to_warp, max_height_image_target, image_to_warp_center,
                      init_target_x, init_target_y, init_angle_deg,
                      verbose=False,
                      target_x_constraints=None,
                      target_y_constraints=None,
                      angle_deg_constraints=None,
                      grid_search=False):

    if target_x_constraints is None: 
        target_x_constraints=[-1000, 1000]
    if target_y_constraints is None: 
        target_y_constraints=[-1000, 1000]
    if angle_deg_constraints is None: 
        angle_deg_constraints=[-360.0, 360.0]
    
    m_per_pix = max_height_image_target.m_per_pix
    m_per_height_unit = max_height_image_target.m_per_height_unit
    image_to_warp = max_height_image_to_warp.image

    weight_by_height = True 

    target_image = max_height_image_target.image
    blur_size = (3,3) 
    image_to_warp = cv2.GaussianBlur(np.float64(image_to_warp), blur_size, 0) 
    target_image_not_smoothed = target_image
    target_image = cv2.GaussianBlur(np.float64(target_image), blur_size, 0)
    target_image[target_image_not_smoothed == 0] = 0

    # As the target image becomes larger than the image being warped
    # this upper bound becomes less meaningful, since it becomes
    # unachievable and feasible values become much smaller.
    # match_score_upper_bound = np.sum(target_image)
    
    # Estimate the upper bound with the image to warp, since it will usually be a smaller local scan.
    match_score_upper_bound = np.sum(image_to_warp)

    image_to_warp_center = np.array(image_to_warp_center)

    target_x_constraints = [init_target_x + c for c in target_x_constraints]
    target_y_constraints = [init_target_y + c for c in target_y_constraints]
    angle_deg_constraints = [init_angle_deg + c for c in angle_deg_constraints]

    print('target_x_constraint =', target_x_constraints)
    print('target_y_constraint =', target_y_constraints)
    print('angle_deg_constraint =', angle_deg_constraints)

    def scale_out(v, min_val, max_val):
        return (v - min_val) / (max_val - min_val)

    def scale_in(v, min_val, max_val):
        return (v * (max_val - min_val)) + min_val
    
    def scale_parameters_in(s):
        # Transform the optimized parameters back to their original
        # forms.
        x = scale_in(s[0], target_x_constraints[0], target_x_constraints[1]) 
        y = scale_in(s[1], target_y_constraints[0], target_y_constraints[1]) 
        a = scale_in(s[2], angle_deg_constraints[0], angle_deg_constraints[1]) 
        return [x,y,a]

    def scale_parameters_out(s):
        # Scale the parameters so that they are between 0 and 1 during
        # the optimization when the constraints are met.
        x = scale_out(s[0], target_x_constraints[0], target_x_constraints[1]) 
        y = scale_out(s[1], target_y_constraints[0], target_y_constraints[1]) 
        a = scale_out(s[2], angle_deg_constraints[0], angle_deg_constraints[1]) 
        return [x,y,a]
        
    def constraints_satisfied(target_x, target_y, angle_deg):
        # Return true if all the constraints are satisfied. Otherwise, return false.
        # When satisfying the constraints, the parameters will be between 0 and 1. 
        target_x_in_range = (0.0 <= target_x) and (target_x <= 1.0)
        target_y_in_range = (0.0 <= target_y) and (target_y <= 1.0)
        angle_deg_in_range = (0.0 <= angle_deg) and (angle_deg <= 1.0)
        return (target_x_in_range and target_y_in_range and angle_deg_in_range)

    def compute_penalty_normalized_input(parameter, scale):
        penalty = 0.0
        if parameter < 0.0:
            penalty += scale * (-1.0 * penalty)
        if parameter > 1.0:
            penalty += scale * (penalty - 1.0)
        return penalty
    
    def constraints_penalty_normalized_input(target_x, target_y, angle_deg):
        # Penalize violation of the constraints. When satisfying the
        # constraints, the parameters will be between 0 and 1.
        scale = 10000.0
        penalty = 0.0
        penalty += compute_penalty_normalized_input(target_x, scale)
        penalty += compute_penalty_normalized_input(target_y, scale)
        penalty += compute_penalty_normalized_input(angle_deg, scale)
        return penalty
    
    def compute_position_penalty(parameter, constraints, scale):
        penalty = 0.0
        if parameter < constraints[0]:
            penalty += scale * abs(parameter - constraints[0])
        if parameter > constraints[1]:
            penalty += scale * abs(parameter - constraints[1])
        return penalty
    
    def compute_angle_penalty(parameter, constraints, scale):
        penalty = 0.0
        if parameter < constraints[0]:
            penalty += scale * abs(parameter - constraints[0])
        if parameter > constraints[1]:
            penalty += scale * abs(parameter - constraints[1])
        return penalty
    
    def constraints_penalty(target_x, target_y, angle_deg):
        # Penalize violation of the constraints. When satisfying the
        # constraints, the parameters will be between 0 and 1.
        scale = 10000.0
        penalty = 0.0
        penalty += compute_position_penalty(target_x, target_x_constraints, scale)
        penalty += compute_position_penalty(target_y, target_y_constraints, scale)
        penalty += compute_angle_penalty(angle_deg, angle_deg_constraints, scale)
        return penalty
    
    def fast_cost_func(s):
        #penalty = constraints_penalty_normalized_input(*s)
        s = scale_parameters_in(s)
        penalty = constraints_penalty(*s)
        target_image_location = s[:2]
        target_x = target_image_location[0]
        target_y = target_image_location[1]
        angle_deg = s[2]
        affine_matrix = cv2.getRotationMatrix2D((image_to_warp_center[0], image_to_warp_center[1]), angle_deg, 1.0)
        affine_matrix[:, 2] += target_image_location - image_to_warp_center
        match_score = numba_compare_images_2(image_to_warp, target_image, target_image_not_smoothed,
                                             affine_matrix, m_per_height_unit, match_threshold_m=0.1,
                                             weight_by_height=True)
        # normalize match_score to be between 0.0 and 1.0
        match_score = match_score/match_score_upper_bound
        cost = penalty - match_score
        return cost

    height, width = target_image.shape
    if init_target_x is None:
        init_target_x = 0.0
    if init_target_y is None:
        init_target_y = 0.0
    if init_angle_deg is None:
        init_angle_deg = 0.0

    if grid_search:
        options = {'tolfun': 0.01}
        initial_standard_deviation = 0.02
        initial_solution = []

        w,h = max_height_image_target.image.shape
        
        # grid of starting positions across the map
        n = 4
        border = w/(2.0*n)
        x_values = np.linspace(border, w - border, n)
        y_values = np.linspace(border, h - border, n)
        a_values = [0.0, 120.0, 240.0]
        
        initial_solution_list = []
        for x in x_values:
            for y in y_values:
                for a in a_values: 
                    initial_solution_list.append([x, y, a])

        print('len(initial_solution_list) =', len(initial_solution_list))
        print('initial_solution_list =', initial_solution_list)

        best_result = None
        for initial_solution in initial_solution_list:
            initial_solution = scale_parameters_out(initial_solution)
            result = cma.fmin(fast_cost_func, initial_solution, initial_standard_deviation, options)
            if best_result is None: 
                best_parameters = scale_parameters_in(result[0])
                best_error = result[1]
                best_result = result
            else:
                new_error = result[1]
                if new_error < best_error:
                    best_parameters = scale_parameters_in(result[0])
                    best_error = result[1]
                    best_result = result

        result = best_result
    else:
        options = {'tolfun': 0.001}
        initial_standard_deviation = 0.02 
        initial_solution = [init_target_x, init_target_y, init_angle_deg]
        initial_solution = scale_parameters_out(initial_solution)
        result = cma.fmin(fast_cost_func, initial_solution, initial_standard_deviation, options)
        best_parameters = scale_parameters_in(result[0])
        best_error = result[1]
        best_result = result

    print
    print('Optimization complete.')
    print
    no_numpy_cma_result = []
    for entry in result:
        if "tolist" in dir(entry):
            entry = entry.tolist()
        no_numpy_cma_result.append(entry)

    cma_result = {'initial_solution': initial_solution,
                  'initial_standard_deviation': initial_standard_deviation,
                  'options': options,
                  'best_parameters': no_numpy_cma_result[0],
                  'best_parameters_error': no_numpy_cma_result[1],
                  'num_evals_to_find_best': no_numpy_cma_result[2],
                  'num_evals_total': no_numpy_cma_result[3],
                  'cma_iterations': no_numpy_cma_result[4],
                  'cma_parameter_means': no_numpy_cma_result[5],
                  'cma_parameter_stddevs': no_numpy_cma_result[6]}

    print('cma_result =')
    print(cma_result)
    
    s_min = scale_parameters_in(cma_result['best_parameters'])
    print('best_parameters =', s_min)
    print('')
    
    return (s_min[0], s_min[1], s_min[2]), cma_result['best_parameters_error']



def transform_xya_to_xya_3d(transform_mat, x, y, ang_rad):
    # Only for 3D affine matrices
    r, c = transform_mat.shape
    assert((r == 3) or (r == 4))
    assert(c == 4)
    x1 = x
    y1 = y
    map_xy_1 = np.matmul(transform_mat, [x1, y1, 0.0, 1.0])[:2]
    a = ang_rad
    f_len = 1.0
    f_x = f_len * np.cos(a)
    f_y = f_len * np.sin(a)
    x2 = x1 + f_x
    y2 = y1 - f_y
    map_xy_2 = np.matmul(transform_mat, [x2, y2, 0.0, 1.0])[:2]
    map_diff = map_xy_2 - map_xy_1
    map_ang_rad = np.arctan2(map_diff[1], map_diff[0])
    return map_xy_1[0], map_xy_1[1], map_ang_rad


def transform_xya_to_xya_2d(transform_mat, x, y, ang_rad):
    # Only for 2D affine matrices
    r, c = transform_mat.shape
    assert((r == 2) or (r == 3))
    assert(c == 3)
    x1 = x
    y1 = y
    map_xy_1 = np.matmul(transform_mat, [x1, y1, 1.0])[:2]
    a = ang_rad
    f_len = 1.0
    f_x = f_len * np.cos(a)
    f_y = f_len * np.sin(a)
    x2 = x1 + f_x
    y2 = y1 - f_y
    map_xy_2 = np.matmul(transform_mat, [x2, y2, 1.0])[:2]
    map_diff = map_xy_2 - map_xy_1
    map_ang_rad = np.arctan2(map_diff[1], map_diff[0])
    return map_xy_1[0], map_xy_1[1], map_ang_rad


def unaligned_merge_scan_1_into_scan_2(scan_1, scan_2, display_on=False, show_unaligned=False):
    mhi_1 = scan_1.max_height_im
    mhi_2 = scan_2.max_height_im
    h1, w1 = mhi_1.image.shape
    h2, w2 = mhi_2.image.shape
    # First, try naively merging without alignment or using camera
    # depth
    if (h1 == h2) and (w1 == w2):
        merged_sub_image = mhi_2.image
    else:
        # This assumes that scan 1 is smaller than scan 2 and is
        # really just a quick hack for testing, since this operation
        # doesn't make much sense if the images have different sizes
        # and aren't aligned.
        merged_sub_image = mhi_2.image[:h1,:w1]
    merged_sub_image[merged_sub_image == 0] = mhi_1.image[merged_sub_image == 0]
    if display_on:
        cv2.imshow('Naive merge', merged.image)

        
def unaligned_blended_scan_1_into_scan_2(scan_1, scan_2, display_on=False, show_unaligned=False):
    mhi_1 = scan_1.max_height_im
    mhi_2 = scan_2.max_height_im
    h1, w1 = mhi_1.image.shape
    h2, w2 = mhi_2.image.shape
    # Second, try merging without alignment, but using camera depth
    if (h1 == h2) and (w1 == w2):
        merged_sub_image = mhi_2.image
        merged_sub_depth_image = mhi_2.camera_depth_image
    else:
        # This assumes that scan 1 is smaller than scan 2 and is
        # really just a quick hack for testing, since this operation
        # doesn't make much sense if the images have different sizes
        # and aren't aligned.
        merged_sub_image = mhi_2.image[:h1,:w1]
        merged_sub_depth_image = mhi_2.camera_depth_image[:h1,:w1]
    # 1 has an observation and 0 does not have an observation.
    unobserved_selector = (merged_sub_image == 0) & (mhi_1.image != 0)
    # 1 has an observation and the camera was closer than in 0. No
    # observation in 0 would result in a camera depth of 0, so unobserved_selector is important.
    nearer_selector = (mhi_1.camera_depth_image < merged_sub_depth_image) & (mhi_1.camera_depth_image != 0)
    selector = unobserved_selector | nearer_selector
    merged_sub_image[selector] = mhi_1.image[selector]
    if display_on:
        cv2.imshow('Unaligned camera depth merge', merged.image)

        
def estimate_scan_1_to_scan_2_transform(scan_1, scan_2, display_on=False, show_unaligned=False,
                                        full_localization=False, init_target=None,
                                        grid_search=False, small_search=False):
    mhi_2 = scan_2.max_height_im
    mhi_1 = scan_1.max_height_im
  
    h, w = mhi_1.image.shape

    if full_localization:
        # Assume no pertinent pose information has been provided, so
        # attempt to search over all feasible poses.
        mhi_to_warp_center = [w/2, h/2]
        min_dim = min(w,h)
        if init_target is None:
            init_target = [w/2, h/2, 0.0]
        # Redundant, but allows some rollover for continuity. Might be a worthwhile tradeoff.
        angle_constraint_deg = 200.0
        position_constraint_pix = min_dim/2
    else:
        print('scan_1.robot_xy_pix = {0}'.format(scan_1.robot_xy_pix))
        print('init_target = {0}'.format(init_target))
        mhi_to_warp_center = [scan_1.robot_xy_pix[0], scan_1.robot_xy_pix[1]]
        if init_target is None:
            init_target = [scan_1.robot_xy_pix[0], scan_1.robot_xy_pix[1], 0.0]
            
        if not small_search: 
            position_constraint_m = 1.2
            angle_constraint_deg = 45.0
        else: 
            position_constraint_m = 0.6
            angle_constraint_deg = 30.0
            
        position_constraint_pix = position_constraint_m / mhi_2.m_per_pix
    mhi_to_warp = mhi_1
    mhi_target = mhi_2

    print('init_target =', init_target)
        
    registration, cost = register_images(mhi_to_warp, mhi_target, mhi_to_warp_center, 
                                         verbose = True,
                                         target_x_constraints=[-position_constraint_pix, position_constraint_pix],
                                         target_y_constraints=[-position_constraint_pix, position_constraint_pix],
                                         angle_deg_constraints=[-angle_constraint_deg, angle_constraint_deg],
                                         init_target_x=init_target[0], init_target_y=init_target[1], init_angle_deg=init_target[2],
                                         grid_search=grid_search)
    target_x, target_y, angle_deg = registration
    print('target_x =', target_x, ', target_y =', target_y, ', angle_deg =', angle_deg)
    affine_matrix = cv2.getRotationMatrix2D((mhi_to_warp_center[0], mhi_to_warp_center[1]), angle_deg, 1.0)
    affine_matrix[:, 2] += np.array((target_x, target_y)) - mhi_to_warp_center    

    # calculate pose of the robot in the new combined map (this
    # assumes that the pose in the map_to_warp is the current pose of
    # the robot)
    map_to_warp_robot_x_pix = scan_1.robot_xy_pix[0]
    map_to_warp_robot_y_pix = scan_1.robot_xy_pix[1]
    map_to_warp_robot_theta_rad = scan_1.robot_ang_rad
    combined_robot_x_pix, combined_robot_y_pix = affine_transform_2d_point(affine_matrix, (map_to_warp_robot_x_pix, map_to_warp_robot_y_pix))
    combined_robot_theta_rad = map_to_warp_robot_theta_rad + ((np.pi/180.0) * angle_deg)
    combined_robot_pose = {'x_pix':combined_robot_x_pix, 'y_pix':combined_robot_y_pix, 'theta_rad':combined_robot_theta_rad}
    print('combined_robot_pose =', combined_robot_pose)

    # Convert to the map frame of reference. This should allow
    # the robot to update its estimate of its pose in the map frame.
    p = combined_robot_pose
    x1 = p['x_pix']
    y1 = p['y_pix']
    a = p['theta_rad']
    map_x, map_y, map_ang_rad = transform_xya_to_xya_3d(scan_2.image_to_map_mat, x1, y1, a)
    map_xy_1 = np.array([map_x, map_y])
    map_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, map_ang_rad)
    print('map_xy_1 =', map_xy_1)
    print('map_ang_rad =', map_ang_rad)
    print('map_quat =', map_quat)
    
    x, y, a = transform_xya_to_xya_3d(scan_2.image_to_map_mat, scan_1.robot_xy_pix[0], scan_1.robot_xy_pix[1], scan_1.robot_ang_rad)

    original_robot_map_pose = [x, y, a]
    corrected_robot_map_pose = [map_x, map_y, map_ang_rad]

    print('original_robot_map_pose =', original_robot_map_pose)
    print('corrected_robot_map_pose =', corrected_robot_map_pose)

    return affine_matrix, original_robot_map_pose, corrected_robot_map_pose


def blend_max_height_image_1_into_max_height_image_2(mhi_1, mhi_2):
    # This assumes that both max height images have camera_depth_images
    assert(mhi_1.camera_depth_image is not None)
    assert(mhi_2.camera_depth_image is not None)
    
    # 1 has an observation and 0 does not have an observation.
    unobserved_selector = (mhi_2.image == 0) & (mhi_1.image != 0)
    # 1 has an observation and the camera was closer than in 0. No
    # observation in 0 would result in a camera depth of 0, so unobserved_selector is important.
    nearer_selector = (mhi_1.camera_depth_image < mhi_2.camera_depth_image) & (mhi_1.camera_depth_image != 0)
    selector = unobserved_selector | nearer_selector
    mhi_2.image[selector] = mhi_1.image[selector]
    mhi_2.camera_depth_image[selector] = mhi_1.camera_depth_image[selector]
    if (mhi_1.rgb_image is not None) and (mhi_2.rgb_image is not None):
        mhi_2.rgb_image[selector] = mhi_1.rgb_image[selector]


def merge_scan_1_into_scan_2(scan_1, scan_2, display_on=False, show_unaligned=False,
                             full_localization=False, init_target=None,
                             grid_search=False, output_affine=False, small_search=False):

    affine_matrix, original_robot_map_pose, corrected_robot_map_pose = estimate_scan_1_to_scan_2_transform(scan_1, scan_2,
                                                                                                           display_on=display_on,
                                                                                                           show_unaligned=show_unaligned,
                                                                                                           full_localization=full_localization,
                                                                                                           init_target=init_target,
                                                                                                           grid_search=grid_search,
                                                                                                           small_search=small_search)
    
    mhi_2 = scan_2.max_height_im
    mhi_1 = scan_1.max_height_im

    mhi_to_warp = mhi_1
    mhi_target = mhi_2

    warped_image_1 = cv2.warpAffine(mhi_to_warp.image, affine_matrix, mhi_to_warp.image.shape, flags=cv2.INTER_NEAREST)
    warped_camera_depth_image_1 = cv2.warpAffine(mhi_to_warp.camera_depth_image, affine_matrix,
                                                 mhi_to_warp.camera_depth_image.shape, flags=cv2.INTER_NEAREST)
    if (mhi_1.rgb_image is not None) and (mhi_2.rgb_image is not None): 
        warped_rgb_image_1 = cv2.warpAffine(mhi_to_warp.rgb_image, affine_matrix,
                                            mhi_to_warp.rgb_image.shape[:2], flags=cv2.INTER_NEAREST)
    else:
        warped_rgb_image_1 = None
    
    if display_on:
        h,w = mhi_target.image.shape
        color_im = np.zeros((h, w, 3), np.uint8)
        color_im[:,:,0] = mhi_target.image
        color_im[:,:,1] = warped_image_1
        cv2.imshow('Aligned color comparison', color_im)
        color_im[:,:,1] = mhi_to_warp.image
        cv2.imshow('Unaligned color comparison', color_im)

    class TempMaxHeightImage:
        image = warped_image_1
        camera_depth_image = warped_camera_depth_image_1
        rgb_image = warped_rgb_image_1

    warped_mhi = TempMaxHeightImage()
    blend_max_height_image_1_into_max_height_image_2(warped_mhi, mhi_2)
    
    if display_on:
        h,w = mhi_2.image.shape
        color_im = np.zeros((h, w, 3), np.uint8)
        color_im[:,:,0] = mhi_2.image
        color_im[:,:,1] = mhi_2.image
        color_im[:,:,2] = mhi_2.image
        x, y, a = corrected_robot_map_pose
        x = int(round(x))
        y = int(round(y))
        radius = 10
        cv2.circle(color_im, (x,y), radius, [0,0,255], 1)
        color_im[y,x] = [0,255,0]
        f_len = 15.0
        f_x = int(round(f_len * np.cos(a)))
        f_y = int(round(f_len * np.sin(a)))
        x2 = x + f_x
        y2 = y - f_y
        cv2.line(color_im, (x, y), (x2, y2), [0,255,255], 1)
        cv2.imshow('Mhi_2 max height image', color_im)

    if not output_affine: 
        return original_robot_map_pose, corrected_robot_map_pose
    else: 
        return original_robot_map_pose, corrected_robot_map_pose, affine_matrix
