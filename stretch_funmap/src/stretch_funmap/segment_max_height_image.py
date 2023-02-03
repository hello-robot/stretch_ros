#!/usr/bin/env python3

import stretch_funmap.max_height_image as mh
import numpy as np
import scipy.ndimage as nd
import scipy.signal as si
import cv2
import skimage as sk
from skimage.morphology import convex_hull_image
import math
import hello_helpers.hello_misc as hm
import stretch_funmap.navigation_planning as na

from stretch_funmap.numba_height_image import numba_create_segment_image_uint8
import hello_helpers.fit_plane as fp


def find_object_to_grasp(height_image, display_on=False):
    h_image = height_image.image
    m_per_unit = height_image.m_per_height_unit
    m_per_pix = height_image.m_per_pix
    height, width = height_image.image.shape
    robot_xy_pix = [width/2, 0]
    surface_mask, plane_parameters = find_closest_flat_surface(height_image, robot_xy_pix, display_on=False)

    if surface_mask is None:
        if display_on: 
            print('No elevated surface found.')
        return None
    
    surface_height_pix = np.max(h_image[surface_mask > 0])
    surface_height_m = m_per_unit * surface_height_pix
    height_image.apply_planar_correction(plane_parameters, surface_height_pix)
    h_image = height_image.image
    if display_on: 
        cv2.imshow('corrected height image', h_image)
        cv2.imshow('rgb image', height_image.rgb_image)

    if display_on: 
        rgb_image = height_image.rgb_image.copy()
        rgb_image[surface_mask > 0] = (rgb_image[surface_mask > 0]/2) + [0, 127, 0] 

    #####################################
    # Select candidate object points

    # Define the minimum height for a candidate object point
    min_object_height_m = 0.01
    min_obstacle_height_m = surface_height_m + min_object_height_m
    min_obstacle_height_pix = min_obstacle_height_m / m_per_unit

    # Define the maximum height for a candidate object point
    robot_camera_height_m = 1.13 #from HeadScan in mapping.py and ManipulationView in manipulation_planning.py)
    voi_safety_margin_m = 0.02 
    max_object_height_m = 0.4
    max_obstacle_height_m = min(robot_camera_height_m - voi_safety_margin_m,
                                surface_height_m + max_object_height_m)
    max_obstacle_height_pix = max_obstacle_height_m / m_per_unit

    # Select candidate object points that are within the valid height range
    obstacle_selector = (h_image > min_obstacle_height_pix) & (h_image < max_obstacle_height_pix)

    if display_on: 
        rgb_image = height_image.rgb_image.copy()
        rgb_image[surface_mask > 0] = (rgb_image[surface_mask > 0]//2) + [0, 127, 0] 
        rgb_image[obstacle_selector] = (rgb_image[obstacle_selector]//2) + [0, 0, 127] 
        cv2.imshow('obstacles', rgb_image)

    obstacle_mask = np.uint8(obstacle_selector)

    if display_on: 
        rgb_image = height_image.rgb_image.copy()
        rgb_image[surface_mask > 0] = (rgb_image[surface_mask > 0]//2) + [0, 127, 0] 
        rgb_image[obstacle_mask > 0] = (rgb_image[obstacle_mask > 0]//2) + [0, 0, 127] 

    # Find the convex hull of the surface points to represent the full
    # surface, overcoming occlusion holes, noise, and other phenomena.
    surface_convex_hull_mask = convex_hull_image(surface_mask)

    # Select candidate object points that are both within the valid
    # height range and on the surface
    obstacles_on_surface_selector = (obstacle_selector & surface_convex_hull_mask) 
    obstacles_on_surface = np.uint8(255.0 * obstacles_on_surface_selector)

    # Dilate and erode the candidate object points to agglomerate
    # object parts that might be separated due to occlusion, noise,
    # and other phenomena.
    kernel_width_pix = 5 #3
    iterations = 3 #5
    kernel_radius_pix = (kernel_width_pix - 1) // 2
    kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
    cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
    use_dilation = True
    if use_dilation:
        obstacles_on_surface = cv2.dilate(obstacles_on_surface, kernel, iterations=iterations)
    use_erosion = True
    if use_erosion:
        obstacles_on_surface = cv2.erode(obstacles_on_surface, kernel, iterations=iterations)

    #####################################
    # Process the candidate object points      
        
    # Treat connected components of candidate object points as objects. Fit ellipses to these segmented objects.
    label_image, max_label_index = sk.measure.label(obstacles_on_surface, background=0, return_num=True, connectivity=2)
    region_properties = sk.measure.regionprops(label_image, intensity_image=None, cache=True)
    if display_on:
        rgb_image = height_image.rgb_image.copy()
        color_label_image = sk.color.label2rgb(label_image, image=rgb_image, colors=None, alpha=0.3, bg_label=0, bg_color=(0, 0, 0), image_alpha=1, kind='overlay')
        cv2.imshow('color_label_image', color_label_image)

    # Proceed if an object was found.
    if len(region_properties) > 0:

        # Select the object with the largest area.
        largest_region = None
        largest_area = 0.0
        for region in region_properties:
            if region.area > largest_area:
                largest_region = region
                largest_area = region.area

        # Make the object with the largest area the grasp target. In
        # the future, other criteria could be used, such as the
        # likelihood that the gripper can actually grasp the
        # object. For example, the target object might be too large.
        object_region = largest_region

        # Collect and compute various features for the target object.
        object_ellipse = get_ellipse(object_region)
        object_area_m_sqr = object_region.area * pow(m_per_pix, 2)
        min_row, min_col, max_row, max_col = object_region.bbox
        object_bounding_box = {'min_row': min_row, 'min_col': min_col, 'max_row': max_row, 'max_col': max_col}

        # Only compute height statistics using the original,
        # high-confidence heights above the surface that are a part of
        # the final object region.
        object_selector = (label_image == object_region.label)
        object_height_selector = obstacles_on_surface_selector & object_selector
        object_heights_m = (m_per_unit * h_image[object_height_selector]) - surface_height_m
        object_mean_height_m = np.mean(object_heights_m)
        object_max_height_m = np.max(object_heights_m)
        object_min_height_m = np.min(object_heights_m)

        if display_on:
            print('object max height = {0} cm'.format(object_max_height_m * 100.0))
            print('object mean height = {0} cm'.format(object_mean_height_m * 100.0))
            rgb_image = height_image.rgb_image.copy()
            #rgb_image[surface_convex_hull_mask > 0] = (rgb_image[surface_convex_hull_mask > 0]/2) + [0, 127, 0]
            rgb_image[surface_convex_hull_mask > 0] = (rgb_image[surface_convex_hull_mask > 0]//2) + [0, 127, 0] 
            rgb_image[label_image == object_region.label] = [0, 0, 255]
            draw_ellipse_axes_from_region(rgb_image, largest_region, color=[255, 255, 255])
            cv2.imshow('object to grasp', rgb_image)

        # ellipse = {'centroid': centroid,
        #            'minor': {'axis': minor_axis, 'length': r.minor_axis_length},
        #            'major': {'axis': major_axis, 'length': r.major_axis_length, 'ang_rad': major_ang_rad}}

        # Prepare grasp target information.
        grasp_location_xy_pix = object_ellipse['centroid']
        major_length_pix = object_ellipse['major']['length']
        major_length_m = m_per_pix * major_length_pix
        minor_length_pix = object_ellipse['minor']['length']
        diff_m = m_per_pix * (major_length_pix - minor_length_pix)
        if display_on:
            print('object_ellipse =', object_ellipse)
        max_gripper_aperture_m = 0.08
        if (diff_m > 0.02) or (major_length_m > max_gripper_aperture_m):
            grasp_elongated = True
            grasp_width_pix = minor_length_pix
            grasp_aperture_axis_pix = object_ellipse['minor']['axis']
            grasp_long_axis_pix = object_ellipse['major']['axis']
        else:
            grasp_elongated = False
            grasp_width_pix = major_length_pix
            grasp_aperture_axis_pix = None
            grasp_long_axis_pix = None

        grasp_width_m = m_per_pix * grasp_width_pix

        fingertip_diameter_m = 0.03
        grasp_location_above_surface_m = max(0.0, object_mean_height_m - (fingertip_diameter_m/2.0))
        grasp_location_z_pix = surface_height_pix + (grasp_location_above_surface_m / m_per_unit)
        
        max_object_height_above_surface_m = object_max_height_m
        
        grasp_target = {'location_xy_pix': grasp_location_xy_pix,
                        'elongated': grasp_elongated,
                        'width_pix' : grasp_width_pix,
                        'width_m' : grasp_width_m,
                        'aperture_axis_pix': grasp_aperture_axis_pix,
                        'long_axis_pix': grasp_long_axis_pix,
                        'location_above_surface_m': grasp_location_above_surface_m,
                        'location_z_pix': grasp_location_z_pix,
                        'object_max_height_above_surface_m': object_max_height_m,
                        'surface_convex_hull_mask': surface_convex_hull_mask,
                        'object_selector': object_selector,
                        'object_ellipse': object_ellipse}

        if display_on:
            print('_________________________________')
            print('grasp_target =')
            print(grasp_target)
            print('_________________________________')

        return grasp_target



def draw_grasp(rgb_image, grasp_target):
    surface_convex_hull_mask = grasp_target['surface_convex_hull_mask']
    object_selector = grasp_target['object_selector']
    object_ellipse = grasp_target['object_ellipse']
    rgb_image[surface_convex_hull_mask > 0] = (rgb_image[surface_convex_hull_mask > 0]//2) + [0, 127, 0] 
    rgb_image[object_selector] = [0, 0, 255]
    draw_ellipse_axes(rgb_image, object_ellipse, color=[255, 255, 255])
    
    
def find_closest_flat_surface(height_image, robot_xy_pix, display_on=False):
    h = height_image
    
    height, width = h.image.shape
    robot_xy_pix = [width/2, 0]

    best_surface = None
    a = None
    
    if display_on: 
        color_im = np.zeros((height, width, 3), np.uint8)
        color_im[:,:,0] = h.image
        color_im[:,:,1] = h.image
        color_im[:,:,2] = h.image

    distance_map = False
    traversable_mask = False

    # segment the max height image
    image = h.image
    m_per_unit = h.m_per_height_unit
    m_per_pix = h.m_per_pix

    # This assumes that the max_height_image is parallel to the
    # floor and that the z component of the volume of interest's
    # origin is defined with respect to a frame_id (f) for which
    # z_f = 0.0 corresponds with the modeled ground plane.
    zero_height = -h.voi.origin[2]
    image_rgb = h.rgb_image

    #~0.1 or ~0.2 for tabletop
    #~0.3 for room 
    segmentation_scale = 0.3

    segments_image, segment_info, height_to_segment_id = segment(image, m_per_unit, zero_height, segmentation_scale, verbose=False)
    if segment_info is None:
        return None, None

    floor_id, floor_mask = find_floor(segment_info, segments_image, verbose=False)
    if floor_mask is None:
        return None, None

    remove_floor = True
    if remove_floor:
        segments_image[floor_mask > 0] = 0

    label_image, max_label_index = sk.measure.label(segments_image, background=0, return_num=True, connectivity=2)
    region_properties = sk.measure.regionprops(label_image, intensity_image=image, cache=True)
    if display_on: 
        color_label_image = sk.color.label2rgb(label_image, image=image_rgb, colors=None, alpha=0.3, bg_label=0, bg_color=(0, 0, 0), image_alpha=1, kind='overlay')


    # manipulation surface area category
    # large
    min_area_m_sqr = 4.0 * (0.1 * 0.1) # 4 x (10 cm squared regions)
    # reachable height
    min_height_m = 0.0
    max_height_m = 0.92

    surfaces = []

    for region in region_properties:
        #area : "Number of pixels of the region."
        label = region.label
        area_m_sqr = region.area * pow(m_per_pix, 2)
        mean_height_m = region.mean_intensity * m_per_unit
        max_height_m = region.max_intensity * m_per_unit
        min_height_m = region.min_intensity * m_per_unit
        yc, xc = region.centroid
        yc = int(round(yc))
        xc = int(round(xc))
        min_row, min_col, max_row, max_col = region.bbox

        if (area_m_sqr >= min_area_m_sqr) and (mean_height_m > min_height_m) and (mean_height_m < max_height_m):
            # Likely a reachable surface region
            surfaces.append({'label': label,
                             'area_m_sqr': area_m_sqr,
                             'mean_height_m': mean_height_m,
                             'max_height_m': max_height_m,
                             'min_height_m': min_height_m,
                             'centroid_x': xc,
                             'centroid_y': yc,
                             'bbox': {'min_row': min_row, 'min_col': min_col, 'max_row': max_row, 'max_col': max_col}})

    surface_label_image = np.zeros_like(label_image, np.uint8)
    for s in surfaces:
        i = s['label']
        surface_label_image[label_image == i] = i
    if display_on: 
        surface_label_image_rgb = sk.color.label2rgb(surface_label_image, image=None, colors=None, alpha=0.3, bg_label=0, bg_color=(0, 0, 0), image_alpha=1, kind='overlay')

        best_surface_image_rgb = np.zeros(surface_label_image_rgb.shape, np.uint8)

    if len(surfaces) > 0: 
        # Find the surface that is closest to the robot, but
        # is not the robot's own body.

        height, width = surface_label_image.shape
        robot_xy_pix = [width/2, 0]
        robot_loc = np.array(robot_xy_pix)
        nearest_x, nearest_y, nearest_surface_label = hm.find_nearest_nonzero(surface_label_image, robot_loc)

        best_surface = np.uint8(surface_label_image == nearest_surface_label)

        #####################

        original_best_surface = best_surface.copy()

        a, X, z = fp.fit_plane_to_height_image(image, best_surface)
        fit_error, z_fit = fp.fit_plane_to_height_image_error(a, X, z)
        fit_error = np.abs(fit_error)
        original_fit_error = fit_error.copy()
        fit_error_m = fit_error * m_per_unit

        min_error_m = np.min(fit_error_m)
        max_error_m = np.max(fit_error_m)
        mean_error_m = np.mean(fit_error_m)
        if display_on: 
            print('-- first fit errors --')
            print('min_error_m =', min_error_m)
            print('max_error_m =', max_error_m)
            print('mean_error_m =', mean_error_m)

        surface_error_threshold_m = 0.02
        surface_points = fit_error_m < surface_error_threshold_m
        surface_temp = best_surface[best_surface > 0]
        surface_temp[~surface_points] = 0
        best_surface[best_surface > 0] = surface_temp

        fit_twice = True
        if fit_twice: 
            a, X, z = fp.fit_plane_to_height_image(image, best_surface)
            fit_error, z_fit = fp.fit_plane_to_height_image_error(a, X, z)
            fit_error = np.abs(fit_error)
            fit_error_m = fit_error * m_per_unit

            min_error_m = np.min(fit_error_m)
            max_error_m = np.max(fit_error_m)
            mean_error_m = np.mean(fit_error_m)
            if display_on: 
                print('-- second fit errors --')
                print('min_error_m =', min_error_m)
                print('max_error_m =', max_error_m)
                print('mean_error_m =', mean_error_m)

            surface_error_threshold_m = 0.01
            surface_points = fit_error_m < surface_error_threshold_m
            surface_temp = best_surface[best_surface > 0]
            surface_temp[~surface_points] = 0
            best_surface[best_surface > 0] = surface_temp

        if display_on: 
            original_max_error = np.max(original_fit_error)
            original_min_error = np.min(original_fit_error)
            display_fit_error = np.uint8(255.0 * ((original_fit_error - original_min_error) / (original_max_error - original_min_error)))
            colors = np.zeros((display_fit_error.shape[0], 3), np.uint8)
            colors[:,2] = display_fit_error
            best_surface_image_rgb[surface_label_image == nearest_surface_label] = colors
            best_surface_image_rgb[best_surface > 0] = [255,0,0]

            radius = 5
            # draw a green circle at the robot's location
            cv2.circle(best_surface_image_rgb, (robot_loc[0], robot_loc[1]), radius, [0, 255, 0], 2)
            # draw a white circle at the point closest to the robot
            cv2.circle(best_surface_image_rgb, (nearest_x, nearest_y), radius, [255, 255, 255], 2)

    if display_on:
        visualize_title = ''
        segments_color_image, key_image = render_segments(segments_image, segment_info, output_key_image=True)
        if key_image is not None: 
            cv2.imshow(visualize_title + 'segmented image key', key_image)
            cv2.imshow(visualize_title + 'segmented image', segments_color_image)
            cv2.imshow(visualize_title + 'color_label_image', color_label_image)
            cv2.imshow(visualize_title + 'surface_label_image_rgb', surface_label_image_rgb)
            cv2.imshow(visualize_title + 'best_surface_image_rgb', best_surface_image_rgb)

    if display_on: 
        cv2.imshow('depth image with detected line', color_im)

    plane_mask = best_surface
    plane_parameters = a
    return plane_mask, plane_parameters
    

def render_segments(segments_image, segment_info, output_key_image=False):
    segment_ids = [segment_id for segment_id in segment_info]
    max_segment_id = max(segment_ids)
    segments_norm = np.uint8(np.round(255.0 * (segments_image/float(max_segment_id))))

    colormap = cv2.COLORMAP_HSV
    segments_color_image = cv2.applyColorMap(segments_norm, colormap)

    selector = (segments_image == 0)
    if np.count_nonzero(selector) > 0:
        segments_color_image[selector] = [0,0,0]
    
    if output_key_image: 
        segment_values = np.sort(np.unique(segments_image))[1:]
        segment_values_norm = np.sort(np.unique(segments_norm)[1:])
        segment_heights_mm = np.array([segment_info[v]['height_m'] for v in segment_values])
        indices = np.argsort(segment_heights_mm)
        segment_heights_mm = segment_heights_mm[indices]
        segment_values_norm = segment_values_norm[indices]
        size = 60
        key_image = np.zeros((size, size * len(segment_values_norm)), np.uint8)
        for (i, v), h in zip(enumerate(segment_values_norm), segment_heights_mm):
            # draw color boxes
            key_image[0:size, i*size:(i+1)*size] = v
        key_image_color = cv2.applyColorMap(key_image, colormap)
        for (i, v), h in zip(enumerate(segment_values_norm), segment_heights_mm):
            # write text on color boxes
            font = cv2.FONT_HERSHEY_PLAIN
            font_scale = 1.1 * (size/100.0)
            line_width = 1
            line_color = (0,0,0)
            cv2.putText(key_image_color, '%.3f m' % h, ((i*size) + size//10, size//2),
                        font, font_scale, line_color, line_width, cv2.LINE_AA)
    else:
        key_image_color = None
    return segments_color_image, key_image_color        


def draw_histogram(hist, image_width, image_height, border=10, color=[255,255,255], bins_to_mark=[]):
    nbins = len(hist)
    max_value = np.max(hist)
    hist_width = image_width - (2 * border)
    hist_height = image_height - (2 * border)
    bin_width = float(hist_width) / float(nbins)
    bin_height = float(hist_height) / float(max_value)
    image = np.zeros((image_height, image_width, 3), dtype=np.uint8)
    for i, v in enumerate(hist):
        ul_x = int(border + round(i * bin_width))
        ul_y = int(image_height - (border + round(v * bin_height)))
        lr_x = int(border + round((i+1) * bin_width))
        lr_y = image_height - border
        if i in bins_to_mark: 
            bin_color = [0, 0, 255]
        else:
            bin_color = color
        cv2.rectangle(image, (ul_x, ul_y), (lr_x, lr_y), bin_color, thickness=-1)
    return image



def visualize_hist(value_hist, segments_image, height_to_segment_id, bins_to_mark=[]):
    hist_image_w = 600
    hist_image_h = 600
    hist_image = draw_histogram(value_hist, hist_image_w, hist_image_h, border=20, bins_to_mark=bins_to_mark)
    cv2.imshow('current histogram used for segmentation', hist_image)
    max_num_distinct_colors = 20.0
    segments_norm = np.uint8(np.round(np.mod((255.0/max_num_distinct_colors) * segments_image, 255.0)))
    colormap = cv2.COLORMAP_HSV
    segments_color = cv2.applyColorMap(segments_norm, colormap)
    segments_color[segments_image==0] = [0,0,0]
    cv2.imshow('current segments', segments_color)

    if np.sum(height_to_segment_id) > 0:
        height_to_segment_id_image = draw_histogram(height_to_segment_id, hist_image_w, hist_image_h, border=20, bins_to_mark=bins_to_mark)
        temp = height_to_segment_id_image/2 + hist_image/2
        cv2.imshow('height to segment id', temp)
    cv2.waitKey(100)


def histogram_segment(segments_image, image,
                      value_hist, bin_edges,
                      segment_info,
                      verbose=False, visualize=False):
    # image: Currently resetricted to be a uint8 max height image.
    
    # There should not be images as large as 20,000 X 20,000 pixels,
    # so this epsilon should represent less than a single normalized
    # vote in the histogram.
    epsilon = 1.0/(20000.0 * 20000.0)
    
    # Number of bins in the histogram.
    num_bins = len(value_hist)

    # 1 dimensional array that represents the segmentation performed
    # over heights. Each possible height has an entry in the
    # array. The value stored at a height's entry is the ID for the
    # segment to which the height belongs. So,
    #
    #           segment_id = height_to_segment_id[height]
    #
    # Currently, this is retricted to 255 distinct segments and a
    # minimum segment ID of 1. For a uint8 max height image, this
    # should be sufficient, since there should not be more distinct
    # segments than distinct heights (i.e., 255 due to 0 representing
    # no observations for a max height image).
    height_to_segment_id = np.zeros(256, np.uint8)

    if verbose: 
        print('value_hist =')
        print(value_hist)

    if visualize:
        visualize_hist(value_hist, height_to_segment_id, segments_image)

    # Segment zero regions. This code assigns contiguous regions of
    # zeros distinct segment IDs. These regions correspond with ranges
    # of heights that were not observed and thus have counts of zero
    # in the height histogram.
    #
    # Example: 
    # [0, 0, 0, 128, 100, 0, 0, 1, 1] => [1, 1, 1, 0, 0, 2, 2, 0, 0]
    #

    value_hist_is_zero = (value_hist <= epsilon)
    zero_segments, num_zero_segments = nd.measurements.label(value_hist_is_zero)

    if verbose: 
        print('zero_segments =', zero_segments)
    
    # Segment bumps. This code assigns contiguous regions between
    # local minima distinct segment IDs. These regions should
    # correpond to bumps. If they were continous functions the second
    # derivative over these regions would <= 0.
    #
    # Find local minima in the histogram. 
    min_bins = si.argrelmin(value_hist)[0]
    # Initialize with ones.
    bumps = np.ones_like(value_hist)
    # Set local minima locations to 0. 
    bumps[min_bins] = 0
    # Set zero regions to 0. 
    bumps[value_hist_is_zero] = 0
    # Segment the remaining regions of contigous 1's. 
    bump_segments, num_bump_segments = nd.measurements.label(bumps)

    if verbose: 
        print('bump_segments =', bump_segments)

    # The local minima have not yet been assigned segment IDs. The
    # goal is to assign each height a segment ID. This code assigns
    # local minima to adjacent segments. The local minima occur at the
    # borders of bump regions. 
    for min_index in min_bins:
        # Set left_index to the index for a bin to the left of the
        # local minimum bin.
        left_index = min_index - 1
        if left_index > 0:
            # If the bin is in bounds, set left_segment_id to be the
            # segment ID for the bump segment to the left of the local
            # minimum bin. If a bump is not on the left side, this
            # will result in setting left_segment_id to be 0.
            left_segment_id = bump_segments[left_index]
        else:
            left_segment_id = None

        # If left_segment_id is valid and corresponds with a segmented
        # region.
        if (left_segment_id is not None) and (left_segment_id != 0):
            # There is a valid bump segment to the left of the local
            # minimum bin. Merge the local mininum bin to the bump
            # segment on its left by setting the local mininum bin's
            # segment ID to the bump segment's ID.
            bump_segments[min_index] = left_segment_id
        else:
            # The local minimum bin does not have a valid bump segment
            # directly to its left. So, attempt to merge the local
            # minimum bin with a bump segment directly to its right.
            right_index = min_index + 1
            if right_index < len(bump_segments):
                # If the bin is in bounds, set right_segment_id to be
                # the segment ID for the bump segment to the right of
                # the local minimum bin. If a bump is not on the right
                # side, this will result in setting right_segment_id
                # to be 0.
                right_segment_id = bump_segments[right_index]
            else:
                right_segment_id = None

            # If right_segment_id is valid and corresponds with a
            # segmented region.
            if (right_segment_id is not None) and (right_segment_id != 0):
                # There is a valid bump segment to the right of the local
                # minimum bin. Merge the local mininum bin to the bump
                # segment on its right by setting the local mininum bin's
                # segment ID to the bump segment's ID.
                bump_segments[min_index] = right_segment_id
            else:
                # No bump segment was found to the left or the right
                # of the local minimum's bin. It's not clear how this
                # would ever happen.
                print('WARNING: histogram_segment - no valid bump segment was found to the left or right of a relative minumum. This may be due to a bug. It also might result in a height region being unassigned to a segment.')

    # Unify the zero region segments and the bump segments into
    # all_segments. This code ensures that the zero segment IDs do not
    # overlap with the bump segment IDs.
    zero_segments[zero_segments != 0] += num_bump_segments
    all_segments = bump_segments
    # Bump segments take priority if there is a conflict with zero
    # segments. Avoid overwriting bump segments by only using zero
    # segments when no bump segment exists.
    all_segments[bump_segments == 0] = zero_segments[bump_segments == 0]
    #all_segments = bump_segments + zero_segments

    if verbose:
        print('all_segments =', all_segments)

    # Check that all of the histogram bins have been assigned to
    # segments. No bin should have a value of zero. Each bin should
    # have a segment ID.
    if not np.alltrue(all_segments > 0):
        print('WARNING: histogram_segment - all_segments has unasigned height bins. This could be due to a bug and is undesirable for a height segmentation.')
        print('         all_segments =', all_segments)

    # For each bin segment, append a dictionary with left_bin and
    # right_bin entries with the leftmost and rightmost bin indices
    # for the bin segment and the index of the bin with the largest
    # weight in the segment. These correspond to the minimum height
    # bin and maximum height bin associated with each segment.
    bin_segments = []

    # Initialize the first information for the first segment.
    i = 0
    bin_segment_id = all_segments[i]
    bin_segment = {'left_bin': i,
                   'max_bin': i,
                   'max_value': value_hist[i],
                   'right_bin': None}

    # Iterate through all of the segment bins.
    for i, v in enumerate(all_segments):
        # Look up the histogram bin associated with the segment bin.
        value = value_hist[i]

        # Check if this is a new bin segment. 
        if bin_segment_id != v:
            # This is a new bin segment, so store the old bin segment
            # and initialize a new bin segment.
            
            # Update and store the previous bin segment.
            bin_segment['right_bin'] = i - 1
            bin_segments.append(bin_segment)
            
            # Initialize a new bin segment.
            bin_segment = {'left_bin': i,
                           'max_bin': i,
                           'max_value': value_hist[i], 
                           'right_bin': None}
            bin_segment_id = v
        else:
            # If the bin value is greater than the current maximum
            # value, update the maximum.
            if value > bin_segment['max_value']:
                bin_segment['max_value'] = value
                bin_segment['max_bin'] = i

    # Update and store the last bin segment.
    bin_segment['right_bin'] = len(all_segments) - 1
    bin_segments.append(bin_segment)

    # Convert the bin segments into height segments.
    segment_id = 1
    image_not_zero = (image != 0)

    # First, iterate through all zero segments in order to update the
    # height to segment id mapping. This ordering is important when
    # the histogram has a higher resolution than the original height
    # image, which is common due to the benefits of using a smoothed
    # high-resolution histogram. This order ensures that non-zero
    # segments take priority and will overwrite zero segments when
    # there is overlapping when mapping from the high-resolution
    # histogram to the original representation.
    for c in bin_segments:
        assert(segment_id <= 255)
        
        # Retrieve the left bin, right bin, and max bin entries
        left_bin = c['left_bin']
        right_bin = c['right_bin']
        max_value = c['max_value']
        max_bin = c['max_bin']
        
        # For a zero segment (i.e., a segment with no associated
        # observations) the max_value will be zero since the entire
        # segment only has zero values. For a zero segment, set the
        # maximum bin to be in the middle of the segment.
        if max_value <= epsilon:
            if verbose:
                print('zero encountered')
            #max_bin = (left_bin + right_bin) / 2
            max_bin = (left_bin + right_bin) // 2

            # Find heights associated with the left of the min bin, the
            # right of the max bin, and center of the most heavily
            # weighted bin.
            min_value = bin_edges[left_bin]
            max_value = bin_edges[right_bin + 1]
            best_value = (bin_edges[max_bin] + bin_edges[max_bin + 1]) / 2.0

            # Record the bin in the segment with the most weight (most
            # observations), its weight, and the height to which it
            # corresponds.
            segment_info[segment_id] = {'bin_index':max_bin, 'bin_value':value_hist[max_bin], 'height_unit':best_value} 
            # Assign the the segment ID to the appropriate range of height_to_segment_id
            #
            min_height = int(round(min_value))
            max_height = int(round(max_value))
            
            # Slicing makes upperbound safe.
            height_to_segment_id[min_height : max_height + 1] = segment_id

            # Increment the segment ID
            segment_id = segment_id + 1
            assert(segment_id <= 255)
            
            if visualize:
                visualize_hist(value_hist, segments_image, height_to_segment_id, [max_bin])

    # Second, iterate through all the non-zero segments in order to
    # update the height to segment id mapping. This ordering is
    # important when the histogram has a higher resolution than the
    # original height image, which is common due to the benefits of
    # using a smoothed high-resolution histogram. This order ensures
    # that non-zero segments take priority and will overwrite zero
    # segments when there is overlapping when mapping from the
    # high-resolution histogram to the original representation.
    for c in bin_segments:
        assert(segment_id <= 255)

        # Retrieve the left bin, right bin, and max bin entries
        left_bin = c['left_bin']
        right_bin = c['right_bin']
        max_value = c['max_value']
        max_bin = c['max_bin']
        
        # For a zero segment (i.e., a segment with no associated
        # observations) the max_value will be zero since the entire
        # segment only has zero values. For a zero segment, set the
        # maximum bin to be in the middle of the segment.
        if max_value > epsilon: 
            # Find heights associated with the left of the min bin, the
            # right of the max bin, and center of the most heavily
            # weighted bin.
            min_value = bin_edges[left_bin]
            max_value = bin_edges[right_bin + 1]
            best_value = (bin_edges[max_bin] + bin_edges[max_bin + 1]) / 2.0

            # Record the bin in the segment with the most weight (most
            # observations), its weight, and the height to which it
            # corresponds.
            segment_info[segment_id] = {'bin_index':max_bin, 'bin_value':value_hist[max_bin], 'height_unit':best_value} 
            # Assign the segment ID to the appropriate range of height_to_segment_id
            min_height = int(round(min_value))
            max_height = int(round(max_value))

            # Slicing makes upperbound safe.
            height_to_segment_id[min_height : max_height + 1] = segment_id

            # Increment the segment ID
            segment_id = segment_id + 1
            
            if visualize:
                visualize_hist(value_hist, segments_image, height_to_segment_id, [max_bin])

    if verbose: 
        print('height_to_segment_id =', height_to_segment_id)
    if False:
        z = (height_to_segment_id == 0)
        test = np.sum(z)
        print('np.sum(height_to_segment_id == 0) :', test)
        if test > 0:
            print('(height_to_segment_id == 0) :', z)
    
    # Create an image for which each pixel holds the segment_id. This
    # implementation requires fewer than 255 distinct segments to
    # work.
    out = numba_create_segment_image_uint8(segments_image, image, height_to_segment_id)
    if out is None:
        print('ERROR: numba_create_segment_image_uint8 failed!')
    
    return height_to_segment_id


def segment(image, m_per_unit, zero_height_m, segmentation_scale=0.1, verbose=False):
    segments_image = np.zeros_like(image)

    mm_per_unit = 1000.0 * m_per_unit
    dist_threshold_mm = 70.0
    dist_threshold = dist_threshold_mm / mm_per_unit
    if verbose: 
        print('dist_threshold =', dist_threshold)

    # use a constant range in an attempt to achieve more consistent
    # segmentations
    #
    # if the range and number of bins is matched to the range of 8 bit
    # values, this could most likely be performed more efficiently
    # with custom code
    range_min = 1
    range_max = 255
    range_width = ((range_max - range_min) + 1)
    # using whole numbers should avoid nonuniform distribution of the
    # bins across the possible values
    whole_number_multiplier = 2 
    num_bins =  whole_number_multiplier * range_width 
    bin_per_unit = num_bins / float(range_width)
    values = image[image!=0].flatten()

    if len(values) <= 0:
        print('*** ERROR: segment called using image with all zero entries ***')
        return None, None, None
    
    value_hist, bin_edges = np.histogram(values, bins=num_bins, range=(range_min, range_max), normed=False, weights=None, density=True)

    scale = segmentation_scale
    sigma = scale * (bin_per_unit * dist_threshold)
    # "... about 68 percent ... are within one standard deviation of
    # the mean ..., about 95 percent are within two standard
    # deviations ..., and about 99.7 percent lie within three standard
    # deviations" - Wikipedia
    # https://en.wikipedia.org/wiki/Standard_deviation
    value_hist = nd.gaussian_filter1d(value_hist, sigma)

    segment_info = {}
    height_to_segment_id = None
    height_to_segment_id = histogram_segment(segments_image, image,
                                             value_hist, bin_edges,
                                             segment_info, verbose)        
    for segment_id in segment_info:
        height_unit = segment_info[segment_id]['height_unit']
        height_m = (m_per_unit * height_unit) - zero_height_m
        segment_info[segment_id]['height_m'] = height_m

    return segments_image, segment_info, height_to_segment_id



def test_segment(image, mm_per_unit, zero_height_unit, segmentation_scale=0.1, verbose=True, visualize=True):
    segments_image = np.zeros_like(image)

    dist_threshold_mm = 70.0
    dist_threshold = dist_threshold_mm / mm_per_unit
    if verbose: 
        print('dist_threshold =', dist_threshold)

    # Create Histogram
    #
    # use a constant range in an attempt to achieve more consistent
    # segmentations
    #
    # if the range and number of bins is matched to the range of 8 bit
    # values, this could most likely be performed more efficiently
    # with custom code
    range_min = 1
    range_max = 255
    range_width = ((range_max - range_min) + 1)
    # using whole numbers should avoid nonuniform distribution of the
    # bins across the possible values
    whole_number_multiplier = 2
    num_bins =  whole_number_multiplier * range_width 
    bin_per_unit = num_bins / float(range_width)
    values = image[image!=0].flatten()
    value_hist, bin_edges = np.histogram(values, bins=num_bins, range=(range_min, range_max), normed=False, weights=None, density=True)
    
    # Simple Bimodal Histogram for Testing
    value_hist[:] = 0
    value_hist[75] = 100.0
    value_hist[175] = 100.0

    scale = segmentation_scale 
    sigma = scale * (bin_per_unit * dist_threshold)
    # "... about 68 percent ... are within one standard deviation of
    # the mean ..., about 95 percent are within two standard
    # deviations ..., and about 99.7 percent lie within three standard
    # deviations" - Wikipedia
    # https://en.wikipedia.org/wiki/Standard_deviation
    value_hist = nd.gaussian_filter1d(value_hist, sigma)

    segment_info = {}
    height_to_segment_id = None
    
    height_to_segment_id = histogram_segment(segments_image, image,
                                             value_hist, bin_edges,
                                             segment_info, verbose=verbose, visualize=visualize)        
    for segment_id in segment_info:
        height_unit = segment_info[segment_id]['height_unit']
        height_m = m_per_unit * (height_unit - zero_height_unit)
        segment_info[segment_id]['height_m'] = height_m

    return segments_image, segment_info, height_to_segment_id


def get_ellipse(region_properties):
    # calculate line segments for ellipse axes
    r = region_properties
    centroid_y, centroid_x = r.centroid
    major_ang_rad = r.orientation

    minor_offset_x = (0.5 * math.sin(major_ang_rad) * r.minor_axis_length)
    minor_offset_y = (0.5 * math.cos(major_ang_rad) * r.minor_axis_length)
    minor_1_x = centroid_x - minor_offset_x
    minor_2_x = centroid_x + minor_offset_x
    minor_1_y = centroid_y - minor_offset_y
    minor_2_y = centroid_y + minor_offset_y

    major_offset_x = (0.5 * math.cos(major_ang_rad) * r.major_axis_length)
    major_offset_y = (0.5 * math.sin(major_ang_rad) * r.major_axis_length)
    major_1_x = centroid_x + major_offset_x
    major_2_x = centroid_x - major_offset_x
    major_1_y = centroid_y - major_offset_y
    major_2_y = centroid_y + major_offset_y

    centroid = (centroid_x, centroid_y)
    minor_axis = ((minor_1_x, minor_1_y), (minor_2_x, minor_2_y))
    major_axis = ((major_1_x, major_1_y), (major_2_x, major_2_y))

    ellipse = {'centroid': centroid,
               'minor': {'axis': minor_axis, 'length': r.minor_axis_length},
               'major': {'axis': major_axis, 'length': r.major_axis_length, 'ang_rad': major_ang_rad}}
    
    return ellipse

def draw_ellipse_axes(image, ellipse, color=[255, 255, 255], draw_line_contrast=True):
    minor_axis = np.int32(np.round(np.array(ellipse['minor']['axis'])))
    major_axis = np.int32(np.round(np.array(ellipse['major']['axis'])))
    if draw_line_contrast: 
        cv2.line(image, tuple(minor_axis[0]), tuple(minor_axis[1]), [0,0,0], 3)
        cv2.line(image, tuple(major_axis[0]), tuple(major_axis[1]), [0,0,0], 3)
    cv2.line(image, tuple(minor_axis[0]), tuple(minor_axis[1]), color, 1)
    cv2.line(image, tuple(major_axis[0]), tuple(major_axis[1]), color, 1)


def draw_ellipse_axes_from_region(image, region_properties, color=[255, 255, 255], draw_line_contrast=True):
    ellipse = get_ellipse(region_properties)
    minor_axis = np.int32(np.round(np.array(ellipse['minor']['axis'])))
    major_axis = np.int32(np.round(np.array(ellipse['major']['axis'])))
    if draw_line_contrast: 
        cv2.line(image, tuple(minor_axis[0]), tuple(minor_axis[1]), [0,0,0], 3)
        cv2.line(image, tuple(major_axis[0]), tuple(major_axis[1]), [0,0,0], 3)
    cv2.line(image, tuple(minor_axis[0]), tuple(minor_axis[1]), color, 1)
    cv2.line(image, tuple(major_axis[0]), tuple(major_axis[1]), color, 1)
    

def draw_text(image, text, x, y): 
    font = cv2.FONT_HERSHEY_PLAIN
    font_scale = 1.0
    line_width = 2
    line_color = (0,0,0)
    cv2.circle(image, (x, y), 4, [0,0,0], -1)
    cv2.putText(image, text, (x-15, y-10),
                font, font_scale, [255,255,255], 1+line_width, cv2.LINE_AA)
    cv2.putText(image, text, (x-15, y-10),
                font, font_scale, line_color, line_width, cv2.LINE_AA)

def full_segment(image, image_rgb, segmentation_scale, m_per_unit, zero_height=0.0, visualize=True, visualize_title='', verbose=False):
    segments_image, segment_info, height_to_segment_id = segment(image, m_per_unit, zero_height, segmentation_scale, verbose)
    #label_image, max_label_index = sk.measure.label(segments_image, neighbors=8, background=0, return_num=True, connectivity=None)
    label_image, max_label_index = sk.measure.label(segments_image, background=0, return_num=True, connectivity=2)
    #region_properties = sk.measure.regionprops(label_image, intensity_image=image, cache=True, coordinates='xy')
    region_properties = sk.measure.regionprops(label_image, intensity_image=image, cache=True)
    if image_rgb is None: 
        color_label_image = sk.color.label2rgb(label_image, image=None, colors=None, alpha=0.3, bg_label=0, bg_color=(0, 0, 0), image_alpha=1, kind='overlay')
    else:
        color_label_image = sk.color.label2rgb(label_image, image=image_rgb, colors=None, alpha=0.3, bg_label=0, bg_color=(0, 0, 0), image_alpha=1, kind='overlay')

    if visualize:
        segments_color_image, key_image = render_segments(segments_image, segment_info, output_key_image=True)
        if key_image is not None: 
            cv2.imshow(visualize_title + 'segmented image key', key_image)
            cv2.imshow(visualize_title + 'segmented image', segments_color_image)
            
    return region_properties, label_image, max_label_index, color_label_image


def compute_floor_mask(max_height_im):
    image = max_height_im.image
    m_per_unit = max_height_im.m_per_height_unit
    m_per_pix = max_height_im.m_per_pix

    # This assumes that the max_height_image is parallel to the
    # floor and that the z component of the volume of interest's
    # origin is defined with respect to a frame_id (f) for which
    # z_f = 0.0 corresponds with the modeled ground plane.
    zero_height = -max_height_im.voi.origin[2]

    # Segment the height image in order to find the floor. 
    #~0.1 or ~0.2 for tabletop scale
    #~0.3 for room scale
    segmentation_scale = 0.3
    segments_image, segment_info, height_to_segment_id = segment(image, m_per_unit, zero_height, segmentation_scale, verbose=False)
    floor_id, floor_mask = find_floor(segment_info, segments_image, verbose=False)

    # Remove small obstacles from the floor mask, including
    # regions that are too far below the floor.
    use_mean = False 
    if use_mean: 
        mean_floor_height_pix = np.mean(max_height_im.image[floor_mask > 0])
        mean_floor_height_m = m_per_unit * mean_floor_height_pix
        print('mean_floor_height_m =', mean_floor_height_m)
        floor_height_pix = mean_floor_height_pix
    else: 
        median_floor_height_pix = np.median(max_height_im.image[floor_mask > 0])
        median_floor_height_m = m_per_unit * median_floor_height_pix
        print('median_floor_height_m =', median_floor_height_m)
        floor_height_pix = median_floor_height_pix

    avoid_small_obstacles = False
    if avoid_small_obstacles: 
        small_obstacle_threshold_m = 0.04
        small_obstacle_selector = ((max_height_im.image > (floor_height_pix + (small_obstacle_threshold_m/m_per_unit))) |
                                   (max_height_im.image < (floor_height_pix - (small_obstacle_threshold_m/m_per_unit))))
        small_obstacle_mask = 255 * np.uint8(small_obstacle_selector)
        floor_mask[small_obstacle_selector] = 0
    return floor_mask


def find_floor(segment_info, segments_image, verbose=False):
    if segment_info is None:
        return None, None
    max_bin_value = 0.0
    floor_id = None
    print('segment_max_height_image.py : find_floor')
    for i in segment_info:
        height_m = segment_info[i]['height_m']
        bin_value = segment_info[i]['bin_value']

        # New values set on June 14, 2022.
        min_floor_m = -0.1
        max_floor_m = 0.1

        if verbose: 
            print('segment = {0}, histogram bin_value = {1}, height_m = {2}, min_floor_m = {3}, max_floor_m = {4}'.format(i, bin_value, height_m, min_floor_m, max_floor_m))
        if (height_m >= min_floor_m) and (height_m <= max_floor_m):
            if verbose: 
                print('found candidate floor segment!')
            if (floor_id is None) and (bin_value > 0.0):
                floor_id = i
                max_bin_value = bin_value
            else:
                if bin_value > max_bin_value:
                    floor_id = i
                    max_bin_value = bin_value
    floor_mask = None
    if floor_id is not None:
        # good floor candidate found
        floor_mask = 255 * np.uint8(segments_image == floor_id)
        if verbose:
            print('Found a good floor candidate.')
            print('floor_id =', floor_id)
            print('max_bin_value =', max_bin_value)
    else:
        print('segment_max_height_image.py : no floor segment found...')
    return floor_id, floor_mask



def process_max_height_image(max_height_im, robot_x_pix, robot_y_pix, robot_ang_rad, display_on = False):
    # Estimate where the robot can navigate given its current pose
    # and and the map.
    distance_map = False
    traversable_mask = False

    # segment the max height image
    image = max_height_im.image
    m_per_unit = max_height_im.m_per_height_unit

    # This assumes that the max_height_image is parallel to the
    # floor and that the z component of the volume of interest's
    # origin is defined with respect to a frame_id (f) for which
    # z_f = 0.0 corresponds with the modeled ground plane.
    zero_height = -max_height_im.voi.origin[2]

    image = max_height_im.image
    image_rgb = max_height_im.rgb_image
    if image.dtype != np.uint8: 
        image = np.uint8(255.0 * (max_height_im.image/np.max(max_height_im)))
    verbose = False

    #~0.1 or ~0.2 for tabletop scale
    #~0.3 for room scale
    segmentation_scale = 0.3

    segments_image, segment_info, height_to_segment_id = segment(image, m_per_unit, zero_height, segmentation_scale, verbose=False)
    floor_id, floor_mask = find_floor(segment_info, segments_image, verbose=True)

    if floor_id is not None:
        m_per_pix = max_height_im.m_per_pix

        # The best case minimum width of the robot in meters when moving forward and backward. 
        min_robot_width_m = 0.3

        if display_on: 
            cv2.imshow('floor mask before drawing robot footprint', floor_mask)

        distance_map, traversable_mask = na.distance_map_simple( floor_mask, m_per_pix, min_robot_width_m, robot_x_pix, robot_y_pix, robot_ang_rad, display_on=display_on, verbose=True )

    visualize_title = ''
    if display_on:
        segments_color_image, key_image = render_segments(segments_image, segment_info, output_key_image=True)
        if key_image is not None: 
            cv2.imshow(visualize_title + 'segmented image key', key_image)
            cv2.imshow(visualize_title + 'segmented image', segments_color_image)
    if display_on: 
        cv2.waitKey(1000)

    return distance_map, traversable_mask
