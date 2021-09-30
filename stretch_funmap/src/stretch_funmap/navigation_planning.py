import copy
import numpy as np
import scipy.ndimage as nd
import cv2
import stretch_funmap.cython_min_cost_path as cm
from stretch_funmap.numba_check_line_path import numba_check_line_path
from stretch_funmap.numba_sample_ridge import numba_sample_ridge, numba_sample_ridge_list
import stretch_funmap.segment_max_height_image as sm
import hello_helpers.hello_misc as hm

####################################################
# DISPLAY FUNCTIONS

def draw_pix_path(pix_path, image, color, radius=4): 
    p_x, p_y = None, None
    for x, y in pix_path:
        cv2.circle(image, (x, y), radius, color, -1)
        if p_x is not None: 
            cv2.line(image, (p_x, p_y), (x, y), color, 1)
        p_x, p_y = x, y

def draw_line_segment_path(image, line_segment_path, line_color=(255,255,255), line_width=1, vertex_radius=5, font_scale=2.0):  
    cv2.polylines(image, [np.array(line_segment_path)], False, line_color)
    for n, p in enumerate(line_segment_path):
        p_x, p_y = p
        cv2.circle(image, (p_x, p_y), vertex_radius, line_color, line_width)
        font = cv2.FONT_HERSHEY_PLAIN
        # Write the vertex number.
        cv2.putText(image, '%d' % n, (p_x, p_y), font, font_scale, line_color, line_width, cv2.LINE_AA)


####################################################
# AUTONOMOUS MAPPING FUNCTIONS
#
# Good places to scan have a number of properties:
#
#  + They are likely to result in observing regions up close that were
#    previously observed from far away.
#
#  + They are likely to observe previously unobserved regions.
#
#  + They are likely to observe previously well-observed regions in
#    order to localize the new scan with respect to old scans.
#
#  + They are likely to be reachable via navigation.
#
#  + They are likely to allow the robot to leave the location after
#    making a scan.
#
#  + They are near the robot's current position, so that it will not
#    need to move far.
#
#  + They have not been previously scanned recently under similar circumstances.
#

def distance_map(max_height_im, robot_xya_pix, floor_mask=None):
    easy_distance_map, traversable_mask = None, None
    
    robot_x_pix = int(round(robot_xya_pix[0]))
    robot_y_pix = int(round(robot_xya_pix[1]))
    robot_ang_rad = robot_xya_pix[2]
    
    # The best case minimum width of the robot in meters when moving forward and backward. 
    min_robot_width_m = 0.34

    m_per_pix = max_height_im.m_per_pix

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

    if floor_mask is None:
        floor_mask = sm.compute_floor_mask(max_height_im)

    # Assume the robot's current footprint is traversable. Draw
    # the robot's footprint as floor on the floor mask.
    draw_robot_footprint_rectangle(robot_x_pix, robot_y_pix, robot_ang_rad, m_per_pix, floor_mask)
    
    if floor_mask is not None:
        # Plan an optimistic path on the floor to the goal pose.
        start_xy = np.array([robot_x_pix, robot_y_pix])
        disallow_too_narrow = True
        easy_distance_map, traversable_mask = distance_map_simple(floor_mask, m_per_pix, min_robot_width_m,
                                                                  robot_x_pix, robot_y_pix, robot_ang_rad,
                                                                  disallow_too_narrow=disallow_too_narrow,
                                                                  display_on=False, verbose=False )
    
    return easy_distance_map, traversable_mask

    
def check_line_path(max_height_im, robot_xya_pix, end_xy_pix, floor_mask=None):    
    m_per_pix = max_height_im.m_per_pix
    
    robot_x_pix = int(round(robot_xya_pix[0]))
    robot_y_pix = int(round(robot_xya_pix[1]))
    robot_ang_rad = robot_xya_pix[2]
    
    start_xy = np.array([robot_x_pix, robot_y_pix])
    
    # ensure that end_xy_pix has the correct type for Cython
    end_xy_pix = np.int64(np.round(np.array(end_xy_pix)))
    
    easy_distance_map, traversable_mask = distance_map(max_height_im, robot_xya_pix, floor_mask=None)
    
    # The best case minimum width of the robot in meters when moving forward and backward. 
    min_robot_width_m = 0.34
    safety_margin_m = 0.02
    distance_threshold_pix = ((min_robot_width_m + safety_margin_m) / 2.0) / m_per_pix
    
    check_result = numba_check_line_path(start_xy[:2], end_xy_pix, easy_distance_map, distance_threshold_pix)
    if check_result is None:
        print('check_line_path: start_xy or end_xy_pix was not inside the distance map.')
        print('                 start_xy = {0}, end_xy_pix = {1}'.format(start_xy, end_xy_pix))
        check_result = False
        
    return check_result


def plan_a_path(max_height_im, robot_xya_pix, end_xy_pix, floor_mask=None):
    # Transform the robot's current estimated pose as represented
    # by TF2 to the map image. Currently, the estimated pose is
    # based on the transformation from the map frame to the
    # base_link frame, which is updated by odometry and
    # corrections based on matching head scans to the map.
    m_per_pix = max_height_im.m_per_pix

    robot_x_pix = int(round(robot_xya_pix[0]))
    robot_y_pix = int(round(robot_xya_pix[1]))
    robot_ang_rad = robot_xya_pix[2]
    
    start_xy = np.array([robot_x_pix, robot_y_pix])
    
    # ensure that end_xy_pix has the correct type for Cython
    end_xy_pix = np.int64(np.round(np.array(end_xy_pix)))
    
    easy_distance_map, traversable_mask = distance_map(max_height_im, robot_xya_pix, floor_mask=None)

    if (easy_distance_map is None) or (traversable_mask is None):
        return None, 'Failed to create distance map and traversable mask.'

    path = find_min_cost_path(easy_distance_map, start_xy, end_xy_pix)
    if path is None:
        message='Unable to find a path to the new head scan location.'
        return path, message

    # Approximate the pixel path on the image map with a line
    # segment path.
    max_error_m = 0.05
    line_segment_path = approximate_with_line_segment_path(path, max_error_m, m_per_pix, verbose=False)
    message = 'Path found!'
    return line_segment_path, message


def head_scan_model(scan_x, scan_y, camera_height_m, max_scan_distance_m, max_height_im, floor_mask):
    # Simple model of what a scan performed at a location on the map
    # will observe.
    
    m_per_pix = max_height_im.m_per_pix
    camera_depth_map = max_height_im.camera_depth_image
    max_height_map = max_height_im.image
    
    circle_image = np.zeros_like(camera_depth_map)
    radius = int(round(max_scan_distance_m / m_per_pix))
    # draw circle over the expected observed area
    cv2.circle(circle_image, (scan_x, scan_y), radius, 1, -1)
    circle_selector = (circle_image == 1) & (floor_mask > 0)
    circle_depths = camera_depth_map[circle_selector]
    # This is a quick approximation. A better model would take into account the height of the
    # observation and the line of sight.

    # Expected scan depth of the floor at the max scan distance.
    max_camera_depth_m = np.sqrt(np.square(max_scan_distance_m) + np.square(camera_height_m))
    max_camera_depth_pix = max_height_im.m_to_camera_depth_pix(max_camera_depth_m)

    # Expected counts of observations of different pixel types
    num_low_quality = np.sum(circle_depths > max_camera_depth_pix)
    num_high_quality = np.sum(circle_depths <= max_camera_depth_pix)
    num_unobserved = np.sum(max_height_map[circle_selector] == 0)
    max_total = np.sum(circle_selector)

    return num_low_quality, num_high_quality, num_unobserved, max_total
    

def create_ridge_mask(distance_map, min_robot_radius_pix):
    # find ridges of the distance map by finding pixels that have
    # higher values than most of pixels surrounding them
    use_stable_values = True
    if use_stable_values:
        # These values have worked pretty well in a number of tests as of 9/20/2018
        neighborhood_size = 17
        subtracted_from_mean = -1.0 
    else:
        # A larger neighborhood of 21 results in more roadmap graph
        # branches, probably too many, but also results in more
        # correctly designated exit vertices.
        neighborhood_size = 23
        subtracted_from_mean = -1.0
    norm_dist_transform = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    max_ridges = cv2.adaptiveThreshold(norm_dist_transform, 255,
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
                                       neighborhood_size, subtracted_from_mean)
    # only keep the largest connected component of the ridges
    max_ridges[distance_map < min_robot_radius_pix] = 0
    only_keep_largest_connected_component = False
    if only_keep_largest_connected_component: 
        max_ridges, component_info = get_largest_component(max_ridges)
        max_ridges[max_ridges > 0] = 255
        max_ridges = np.uint8(max_ridges)

    return max_ridges


def find_good_scan_candidates(location_list, camera_height_m,
                              max_scan_distance_m, max_height_im, floor_mask, distance_map, 
                              robot_x_pix, robot_y_pix):

    m_per_pix = max_height_im.m_per_pix
    good_scan_candidates = []
    for x,y in location_list:
        num_low_quality, num_high_quality, num_unobserved, max_total = head_scan_model(x, y, camera_height_m,
                                                                                       max_scan_distance_m, 
                                                                                       max_height_im, floor_mask)
        # percentage of the scan area that has previously been observed
        # previous high quality observations for localization / low quality observations that could become high quality
        if max_total > 0: 
            high_quality_percent = num_high_quality / float(max_total)
            low_quality_percent = num_low_quality / float(max_total)
            if num_low_quality > 0:
                high_to_low_ratio = num_high_quality / float(num_low_quality)
                straight_line_distance = np.sqrt(np.square(x - robot_x_pix) + np.square(y - robot_y_pix))
                obstacle_distance_m = distance_map[y,x] * m_per_pix
                if (high_quality_percent > 0.5) and (low_quality_percent > 0.2) and (high_to_low_ratio > 0.5):
                    good_scan_candidates.append([[x,y], num_low_quality, num_high_quality, num_unobserved, max_total, straight_line_distance, obstacle_distance_m])
                else:
                    pass
            else:
                pass
        else:
            pass

    return good_scan_candidates


def find_best_scan_candidate(good_scan_candidates):
    if len(good_scan_candidates) <= 0:
        return None
    
    # sort by straight line distance
    def sort_by_straight_line_distance(candidate):
        return candidate[5]
    good_scan_candidates.sort(key=sort_by_straight_line_distance)
    nearby_scan_candidates = good_scan_candidates[:6]
    def sort_by_obstacle_distance_m(candidate):
        return candidate[6]
    nearby_scan_candidates.sort(key=sort_by_obstacle_distance_m, reverse=True)
    if len(nearby_scan_candidates) <= 0:
        return None
    best_candidate = nearby_scan_candidates[0]
    
    if False:
        best_xy = None
        best_value = 0
        best_candidate = None
        for c in good_scan_candidates:
            # Select filtered candidate with the largest number of
            # expected low quality map pixels, which are expected
            # to become high quality map pixels after the
            # scane. This is a form of conservative expected
            # information gain. Unobserved pixels are not
            # considered, although they could potentially become
            # low or high quality pixels after the scan.
            #
            # This should also take into account where past scans
            # have been performed to avoid a loop resulting in
            # multiple scans at approximately the same location
            # due to some sort of unmodeled occlusion or other
            # problem.
            value = c[1]
            if value > best_value:
                best_value = value
                best_candidate = c
                
    return best_candidate


def select_next_scan_location(floor_mask, max_height_im, min_robot_width_m,
                              robot_x_pix, robot_y_pix, robot_ang_rad,
                              camera_height_m, max_scan_distance_m,
                              display_on=False):
    h, w = max_height_im.image.shape
    m_per_pix = max_height_im.m_per_pix
    min_robot_radius_pix = (min_robot_width_m / 2.0) / m_per_pix
    disallow_too_narrow = False
    easy_distance_map, traversable_mask = distance_map_simple( floor_mask, m_per_pix, min_robot_width_m,
                                                               robot_x_pix, robot_y_pix, robot_ang_rad,
                                                               disallow_too_narrow=disallow_too_narrow,
                                                               display_on=display_on, verbose=True )
    
    ## MAX RIDGES
    # find ridges of the distance map by finding pixels that have
    # higher values than most of pixels surrounding them
    max_ridges = create_ridge_mask(easy_distance_map, min_robot_radius_pix)
    if display_on:
        cv2.imshow('max_ridges', halve_image(max_ridges))


    ## Sample the ridges of the floor distance map. What if an area of
    ## the floor is very large relative to the range of the sensor?
    window_width = 50 
    norm_easy_distance_map = cv2.normalize(easy_distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    accessible_floor_mask = np.zeros((h+2, w+2), np.uint8)
    cv2.floodFill(floor_mask, accessible_floor_mask, (robot_x_pix, robot_y_pix), 255)
    accessible_floor_mask = 255 * accessible_floor_mask[1:-1,1:-1]
    if False: 
        max_filtered_map = nd.maximum_filter(norm_easy_distance_map, size=window_width, mode='constant')
        # Select the connected component of the floor on which the
        # robot is located.
        max_sample_selector = (max_filtered_map == norm_easy_distance_map) & (accessible_floor_mask > 0)
        max_samples = 255 * np.uint8(max_sample_selector)
    else:
        max_samples = numba_sample_ridge(window_width, max_ridges, norm_easy_distance_map, min_robot_radius_pix)
        max_samples_list = numba_sample_ridge_list(window_width, max_ridges, norm_easy_distance_map, min_robot_radius_pix)
    if display_on: 
        cv2.imshow('max_samples', max_samples)

    good_scan_candidates = find_good_scan_candidates(max_samples_list, camera_height_m,
                                                        max_scan_distance_m, 
                                                        max_height_im, floor_mask,
                                                        easy_distance_map, 
                                                        robot_x_pix, robot_y_pix)
    best_scan_candidate = find_best_scan_candidate(good_scan_candidates)
    if best_scan_candidate is None:
        best_xy = None
    else: 
        best_xy = best_scan_candidate[0][:]

    return best_xy


####################################################
# FLOOR FUNCTIONS

def simple_connected_components(binary_image, connectivity=4):
    output_image_type = cv2.CV_16U
    number_of_components, label_image = cv2.connectedComponents(binary_image, None, connectivity, output_image_type)
    return number_of_components, label_image


def estimate_navigation_channels( floor_mask, idealized_height_image, m_per_pix, display_on=False, verbose=False ):
    # min_robot_width_m : The best case minimum width of the robot in meters when moving forward and backward. 

    # threshold based on minimum radius of the robot
    min_robot_radius_m = 0.4
    # subtract pixels to provide an optimistic estimate and account for possible quantization
    min_robot_radius_pix = (min_robot_radius_m / m_per_pix) - 2.0

    # create optimistic traversable region mask
    traversable_image = np.zeros_like(floor_mask, dtype=np.uint8)
    # if the region is floor or unobserved, consider it to be traversable
    traversable_selector = (floor_mask > 0) | (idealized_height_image == 0)
    traversable_image[traversable_selector] = 255

    if display_on: 
        cv2.imshow('traversable image', traversable_image)

    # compute distance map: distance from untraversable regions
    original_dist_transform = cv2.distanceTransform(traversable_image, cv2.DIST_L2, 5)
    dist_transform = original_dist_transform.copy()
    norm_dist_transform = cv2.normalize(dist_transform, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    if display_on: 
        cv2.imshow('distance transform of the traversable image', norm_dist_transform)

    ########
    # remove unobserved and untraversable pixels from the distance map
    
    # set parts of the distance transform that are off of the observed
    # floor to zero
    floor_selector = (floor_mask < 1)
    dist_transform[floor_selector] = 0

    # set parts of the distance transform that represent free space
    # less than the robot would require to zero
    robot_radius_selector = (dist_transform < min_robot_radius_pix)
    dist_transform[robot_radius_selector] = 0
    if display_on: 
        norm_dist_transform = cv2.normalize(dist_transform, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cv2.imshow('cropped distance transform', norm_dist_transform)
    ########

    # create kernel for morphological operations
    kernel_width_pix = 11
    kernel_radius_pix = (kernel_width_pix - 1) // 2
    kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
    cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
    
    # fill in floor mask holes
    closed_floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_CLOSE, kernel)

    # find the boundary of the floor
    dilated_floor = cv2.dilate(closed_floor_mask, kernel, iterations = 1)
    floor_boundary = dilated_floor - closed_floor_mask

    # Estimate the locations of exits. Creates a binary image with exits marked with 255.
    min_connectivity_dist_m = min_robot_radius_m
    min_connectivity_dist_pix = min_robot_radius_pix 
    connectivity_image = original_dist_transform.copy()
    connectivity_image[connectivity_image < min_connectivity_dist_pix] = 0
    connectivity_image[floor_boundary == 0] = 0
    map_exits = np.zeros_like(connectivity_image, dtype=np.uint8)
    map_exits[connectivity_image > 0] = 255
    exit_dilation = True
    if exit_dilation:
        # attempt to increase the chance of a vertex being labeled as an exit
        kernel = np.ones((21,21), np.uint8)
        map_exits = cv2.dilate(map_exits, kernel, iterations = 1)

    # find exit regions
    number_of_exits, exit_label_image = simple_connected_components(map_exits)
    print('number_of_exits =', number_of_exits)

    distance_map = original_dist_transform.copy()
    distance_map[closed_floor_mask < 1] = 0

    exit_locations = []
    for i in range(number_of_exits)[1:]:
        distance = distance_map.copy()
        not_component_selector = (exit_label_image != i)
        distance[not_component_selector] = 0
        y, x = np.unravel_index(np.argmax(distance), distance.shape)
        exit_locations.append([x,y])
        
    if display_on: 
        norm_connectivity_image = cv2.normalize(connectivity_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cv2.imshow('connectivity image', norm_connectivity_image)
        cv2.imshow('estimated map exits', map_exits)
        cv2.imshow('floor boundary', floor_boundary)
        norm_distance_map = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cv2.imshow('raw distance map', norm_distance_map)
        
    return dist_transform, map_exits, distance_map, exit_locations


def draw_robot_mast_blind_spot_wedge(x_pix, y_pix, ang_rad, m_per_pix, image, verbose=False, value=255):
    # The x axis points to the front of the robot.
    # The y axis points to the left of the robot.
    
    # 0.18 m measured diagonal from origin (center of laser range
    # finder) to the edge of the robot that avoids the carriage corner
    #
    # 0.165 m measured across from origin (center of laser range finder) to robot side
    #
    # In [6]: math.acos(0.165/0.18)
    # Out[6]: 0.4111378623223475
    # ~23.5 degrees
    # 1.980795 = 0.41 + 1.570795
    #start_angle = 1.98 + ang_rad

    # larger angular range to account for camera looking down and
    # being closer to the center of the robot
    # 0.54 rad is about 30 deg
    # 1.98 - 0.54 = 1.44
    start_angle = 1.44 + ang_rad
    

    # 0.23 m measured diagonal from origin (center of laser range
    # finder) to edge of the robot.
    #
    # In [5]: math.acos(0.165/0.23)
    # Out[5]: 0.7707457827651633
    # ~44.1 degrees
    # 2.341795 = 0.771 + 1.570795
    end_angle = 2.35 + ang_rad

    # 2.35 - 1.98 = 0.37 rad wedge
    # ~21.2 degree wedge
    # ~6% of a circle

    # Find the maximum possible distance represented in the image to
    # ensure that the blind spot goes to the edge of the image.
    h, w = image.shape[:2]
    max_dist = np.sqrt(np.square(h) + np.square(w))

    x_1 = x_pix
    y_1 = y_pix

    x_2 = (max_dist * np.cos(start_angle)) + x_pix
    y_2 = (-max_dist * np.sin(start_angle)) + y_pix

    x_3 = (max_dist * np.cos(end_angle)) + x_pix
    y_3 = (-max_dist * np.sin(end_angle)) + y_pix
    
    corners = np.array([[x_1, y_1], [x_2, y_2], [x_3, y_3]])
    if verbose: 
        print('corners =', corners)

    poly_points = np.array(corners)
    poly_points = np.round(poly_points).astype(np.int32)
    cv2.fillConvexPoly(image, poly_points, value)
    

def draw_robot_footprint_rectangle(x_pix, y_pix, ang_rad, m_per_pix, image, verbose=False, value=255):
    # One issue to consider is what to do when the mobile base is
    # underneath a surface, such as a table. In these situations,
    # calling this function will erase part of the surface and replace
    # it with floor. This can be problematic, since some robot actions
    # such as stowing or lifting the arm can collide with the surface
    # when mistaking it for floor.
    
    # Robot measurements for rectangular approximation. These should
    # be updated if the robot's footprint changes.
    safety_border_m = 0.02

    # stretch's arm can extend from the side, which adds width to footprint.
    # 0.335 m for the mobile base
    # 0.04 m for the wrist extending over the edge
    # total = 0.335 + (2 * 0.04) due to enforcing symmetry around point of rotation
    robot_width_m = 0.415

    # larger, more conservative model when tethered
    # 0.32 m for the length of the mobile base
    # 0.01 m for the arm E-chain cartridge extending off the back
    # 0.2 m to ignore cables when tethered
    # 0.52 = 0.32 + 0.2
    #robot_length_m = 0.52
    
    # smaller, more optimistic model when untethered
    # 0.32 m for the length of the mobile base
    # 0.01 m for the arm E-chain cartridge extending off the back
    # 0.2 m to ignore cables when tethered
    # 0.33 = 0.32 + 0.01
    robot_length_m = 0.33
    
    origin_distance_from_front_pix_m = 0.05
    
    robot_width_pix = (robot_width_m + (2.0 * safety_border_m))/m_per_pix
    robot_length_pix = (robot_length_m + (2.0 * safety_border_m))/m_per_pix
    origin_distance_from_front_pix = (origin_distance_from_front_pix_m + safety_border_m)/m_per_pix

    # vector to left side of robot
    ls_ang_rad = ang_rad + (np.pi/2.0)
    ls_x = (robot_width_pix/2.0) * np.cos(ls_ang_rad)
    ls_y = -(robot_width_pix/2.0) * np.sin(ls_ang_rad)

    # vector to front of robot
    f_x = origin_distance_from_front_pix * np.cos(ang_rad)
    f_y = -origin_distance_from_front_pix * np.sin(ang_rad)

    # vector to back of robot
    dist_to_back = robot_length_pix - origin_distance_from_front_pix
    b_x = -dist_to_back * np.cos(ang_rad)
    b_y = dist_to_back * np.sin(ang_rad)

    # front left corner of the robot
    fl_x = x_pix + f_x + ls_x
    fl_y = y_pix + f_y + ls_y

    # front right corner of the robot
    fr_x = (x_pix + f_x) - ls_x
    fr_y = (y_pix + f_y) - ls_y
    
    # back left corner of the robot
    bl_x = x_pix + b_x + ls_x
    bl_y = y_pix + b_y + ls_y

    # back right corner of the robot
    br_x = (x_pix + b_x) - ls_x
    br_y = (y_pix + b_y) - ls_y

    corners = np.array([[fl_x, fl_y], [fr_x, fr_y], [br_x, br_y], [bl_x, bl_y]])
    if verbose: 
        print('corners =', corners)
        
    poly_points = np.array(corners)
    poly_points = np.round(poly_points).astype(np.int32)
    
    if image is not None:
        cv2.fillConvexPoly(image, poly_points, value)

def halve_image(image):
    h, w = image.shape
    scale_divisor = 2
    nh = h/scale_divisor
    nw = w/scale_divisor
    return cv2.resize(image, (nw, nh))

def find_exits( floor_mask, max_height_image, m_per_pix,
                min_robot_width_m, robot_x_pix, robot_y_pix, display_on=False ):
    # A mask with optimistic traversable pixels consisting of floor
    # and unobserved pixels. This should result in traversable paths
    # that move from observed floor to unobserved floor, which can be
    # used to find exits.
    traversable_selector = (floor_mask > 0) | (max_height_image == 0)
    traversable_mask = 255 * np.uint8(traversable_selector)

    # Fill in small non-floor regions (likely noise)

    # TODO: improve this. For example, fill in isolated pixels that
    # are too small to trust as obstacles. Options include a hit or
    # miss filter, matched filter, or speckle filter.
    fill_in = True
    if fill_in:
        kernel = np.ones((3,3), np.uint8)
        traversable_mask = cv2.morphologyEx(traversable_mask, cv2.MORPH_CLOSE, kernel)

    # Optimistic estimate of robot width. Use ceil to account for
    # possible quantization. Consider adding a pixel, also.
    min_robot_radius_pix = np.ceil((min_robot_width_m/2.0) / m_per_pix)
            
    # compute distance map: distance from untraversable regions
    #
    # cv2.DIST_L2 : Euclidean distance
    #
    # 5 is the mask size : "finds the shortest path to the nearest
    # zero pixel consisting of basic shifts: horizontal, vertical,
    # diagonal, or knight's move (the latest is available for a 5x5
    # mask)" - OpenCV documentation
    distance_map = cv2.distanceTransform(traversable_mask, cv2.DIST_L2, 5)
    
    # fill in floor mask holes
    kernel_width_pix = 11
    kernel_radius_pix = (kernel_width_pix - 1) // 2
    kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
    cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
    closed_floor_mask = cv2.morphologyEx(floor_mask, cv2.MORPH_CLOSE, kernel)

    # find the boundary of the floor
    dilated_floor = cv2.dilate(closed_floor_mask, kernel, iterations = 1)
    floor_boundary = dilated_floor - closed_floor_mask
    
    # Estimate the locations of exits. Creates a binary image with exits marked with 255.
    min_connectivity_dist_pix = min_robot_radius_pix #(min_connectivity_dist_mm / mm_per_pix) - 2.0
    connectivity_image = distance_map.copy()
    # remove regions that are too narrow for the robot to drive through
    connectivity_image[connectivity_image < min_connectivity_dist_pix] = 0
    # Only keep pixels that are at the boundary of the observed
    # floor. This should cut between the observed and unobserved
    # traversible regions.
    connectivity_image[floor_boundary == 0] = 0
    # convert float image to uint8 image
    map_exits = np.zeros_like(connectivity_image, dtype=np.uint8)
    # exit pixels have a value of 255
    map_exits[connectivity_image > 0] = 255
    # enlarge the map exit pixels so that they will intersect with the floor
    exit_dilation = True
    if exit_dilation:
        # attempt to increase the chance of a vertex being labeled as an exit
        # create kernel for morphological operations
        kernel_width_pix = 11
        #kernel_radius_pix = (kernel_width_pix - 1) / 2
        kernel_radius_pix = (kernel_width_pix - 1) // 2
        kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
        cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
        map_exits = cv2.dilate(map_exits, kernel, iterations = 1)
        
    # Select the connected component of the floor on which the robot
    # is located.
    h, w = closed_floor_mask.shape
    accessible_floor_mask = np.zeros((h+2, w+2), np.uint8)
    #possibly add to floodFill in the future: flags = cv2.FLOODFILL_FIXED_RANGE
    cv2.floodFill(floor_mask, accessible_floor_mask, (robot_x_pix, robot_y_pix), 255)
    accessible_floor_mask = 255 * accessible_floor_mask[1:-1,1:-1]
    
    # only consider map exit candidates that are connected to parts of
    # the floor that can be reached
    map_exits[accessible_floor_mask == 0] = 0

    # Ignore exits that are very close to the robot. These can be due
    # to the robot's own body occluding the floor. Otherwise, they can
    # represent real exits that the robot is already next to and does
    # not need to navigate to.
    ignore_radius_pix = int(4.0 * min_robot_radius_pix)
    cv2.circle(map_exits, (robot_x_pix, robot_y_pix), ignore_radius_pix, 0, -1)

    # Dilate exits in order to merge exits that are very close to one
    # another.
    kernel_width_pix = 21 #11 #15
    #kernel_radius_pix = (kernel_width_pix - 1) / 2
    kernel_radius_pix = (kernel_width_pix - 1) // 2
    kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
    cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
    map_exits = cv2.dilate(map_exits, kernel, iterations = 1)

    # only consider map exit candidates that are connected to parts of
    # the floor that can be reached
    map_exits[accessible_floor_mask == 0] = 0
    
    # find exit regions
    number_of_exits, exit_label_image = simple_connected_components(map_exits, connectivity=8)

    # find geometric centers of the exits
    label_indices = range(number_of_exits)[1:]
    label_image = exit_label_image
    centers_of_mass = nd.measurements.center_of_mass(label_image, label_image, label_indices)
    ones = np.ones_like(label_image)
    sums = nd.measurements.sum(ones, label_image, label_indices)
    print('centers_of_mass =', centers_of_mass)

    if display_on:
        print('find_exits: number_of_exits =', number_of_exits)
        h, w = max_height_image.shape
        color_im = np.zeros((h, w, 3), np.uint8)
        color_im[:,:,0] = max_height_image
        color_im[:,:,1] = max_height_image
        color_im[:,:,2] = map_exits
        for s, (c_y, c_x) in zip(sums, centers_of_mass):
            if s > 50:
                c_x = int(round(c_x))
                c_y = int(round(c_y))
                radius = 5 #3
                cv2.circle(color_im, (c_x, c_y), radius, [255, 255, 255], -1)
        scale_divisor = 2
        nh = h/scale_divisor
        nw = w/scale_divisor
        color_im = cv2.resize(color_im, (nw, nh))
        cv2.imshow('find_exits: exits on the map', color_im)

    map_exits_mask = map_exits

    min_area = 50
    exit_points = [[int(round(c_x)), int(round(c_y))] for s, (c_y, c_x) in zip(sums, centers_of_mass) if s > min_area]
    
    return exit_points, map_exits_mask, number_of_exits, exit_label_image
    
        
            
def distance_map_simple( floor_mask, m_per_pix, min_robot_width_m,
                         robot_x_pix, robot_y_pix, robot_ang_rad,
                         disallow_too_narrow=True,
                         display_on=False, verbose=False ):
    
    # min_robot_width_m : The best case minimum width of the robot in meters when moving forward and backward. 
    traversable_mask = floor_mask

    # model the robot's footprint as being traversable
    draw_robot_footprint_rectangle(robot_x_pix, robot_y_pix, robot_ang_rad, m_per_pix, traversable_mask)
    footprint_test_image = np.zeros_like(traversable_mask)
    draw_robot_footprint_rectangle(robot_x_pix, robot_y_pix, robot_ang_rad, m_per_pix, footprint_test_image)
    if display_on:
        cv2.imshow('robot footprint drawing', footprint_test_image)
        cv2.imshow('floor mask after drawing robot footprint', traversable_mask)
    
    # Optimistic estimate of robot width. Use ceil to account for
    # possible quantization. Consider adding a pixel, also.
    min_robot_radius_pix = np.ceil((min_robot_width_m/2.0) / m_per_pix)

    # Fill in small non-floor regions (likely noise)
    #
    # TODO: improve this. For example, fill in isolated pixels that
    # are too small to trust as obstacles. Options include a hit or
    # miss filter, matched filter, or speckle filter.
    fill_in = True
    if fill_in:
        kernel = np.ones((3,3), np.uint8)
        traversable_mask = cv2.morphologyEx(traversable_mask, cv2.MORPH_CLOSE, kernel)
        if display_on:
            cv2.imshow('traversable_mask after filling', traversable_mask)

    # ERROR? : The floodfill operation should occur after removing
    # filtering candidate robot poses due to the footprint
    # radius. Right? Was that too aggressive in the past, so it got
    # dropped?
            
    # Select the connected component of the floor on which the robot
    # is located.
    h, w = traversable_mask.shape
    new_traversable_mask = np.zeros((h+2, w+2), np.uint8)
    #possibly add to floodFill in the future: flags = cv2.FLOODFILL_FIXED_RANGE
    cv2.floodFill(traversable_mask, new_traversable_mask, (robot_x_pix, robot_y_pix), 255)
    traversable_mask = 255 * new_traversable_mask[1:-1,1:-1]
            
    # In previous versions, the traversability mask has treated
    # unobserved pixels and observed non-floor pixels
    # differently. Such as by treating unobserved pixels
    # optimistically as traversable.
            
    # compute distance map: distance from untraversable regions
    #
    # cv2.DIST_L2 : Euclidean distance
    #
    # 5 is the mask size : "finds the shortest path to the nearest
    # zero pixel consisting of basic shifts: horizontal, vertical,
    # diagonal, or knight's move (the latest is available for a 5x5
    # mask)" - OpenCV documentation
    distance_map = cv2.distanceTransform(traversable_mask, cv2.DIST_L2, 5)
    if display_on:
        norm_dist_transform = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)        
        cv2.imshow('distance map without threshold for the robot width', norm_dist_transform)

    # Restricts the maximum distance of the distance_map. This will
    # favor shorter paths (i.e., straight line paths) when a corridor
    # is wide enough instead of moving to the middle of the
    # corridor. When the corridor is narrower than the threshold, the
    # robot will prefer paths that move it to the center of the
    # corridor. However, simple path planning via 4 connected grid and
    # Dijkstra's algorithm results in vertical and horizontal motions
    # in flat regions rather than point-to-point straight lines.
    clip_max_distance = False
    if clip_max_distance:
        max_distance = 3.0 * min_robot_radius_pix
        print('max_distance =', max_distance)
        print('np.max(distance_map) =', np.max(distance_map))
        # should perform in place clipping
        np.clip(distance_map, None, max_distance, distance_map)
        print('after clipping np.max(distance_map) =', np.max(distance_map))
        if display_on:
            norm_dist_transform = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)        
            cv2.imshow('distance map with clipped maximum distance', norm_dist_transform)

    if disallow_too_narrow: 
        # set parts of the distance transform that represent free space
        # less than the robot would require to zero
        distance_map[distance_map < min_robot_radius_pix] = 0
        if display_on: 
            norm_dist_transform = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            cv2.imshow('distance map with robot width threshold', norm_dist_transform)

    # traversable_mask is a binary image that estimates where the
    # robot can navigate given the robot's current pose and the map,
    # but ignoring the robot's radius.

    # distance_map is a scalar image that estimates the distance to
    # the boundaries of the traversable mask.
    return distance_map, traversable_mask
        


####################################################
# PATH FUNCTIONS

def find_min_cost_path(distance_map, start_xy_pix, end_xy_pix):
    # The returned path consists of a list of pixel coordinates in the
    # distance_map, path = [[x0,y0], [x1,y1], ...], that define a
    # minimum cost path from pixel coordinate start_xy to pixel
    # coordinate end_xy. The total cost of a path is the sum of the
    # costs associated with the individual pixel coordinates. Each of
    # these individual costs is a function of the value distance_map
    # at the pixel coordinate. Details can be found in the
    # documentation for cython_min_cost_path.
    # directly plans a path using a distance map

    # first attempt to find a path only traversing regions that are likely traversable
    path = cm.cython_min_cost_path(distance_map, start_xy_pix, end_xy_pix)

    return path


def approximate_with_line_segment_path(path, max_error_m, m_per_pix, verbose=False):
    # Approximate the provided path with line segments such that the
    # maximum distance between the approximated path and the original
    # path is less than max_error_m. The output has the following
    # form, which consists of a list of the vertices between the line
    # segments.
    #
    # line_segment_path = [[x0,y0], [x1,y1], ...]
    #

    #
    # cv2.approxPolyDP(contour, epsilon, closed=False)
    #
    #"epsilon, which is maximum distance from contour to
    #approximated contour" -
    #https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html
    #
    #"epsilon : Parameter specifying the approximation
    #accuracy. This is the maximum distance between the
    #original curve and its approximation." -
    #https://docs.opencv.org/4.0.1/d3/dc0/group__imgproc__shape.html#ga0012a5fdaea70b8a9970165d98722b4c
    #
    
    max_error_pix = max_error_m / m_per_pix
    contour = np.array(path, dtype=np.int32)
    line_segment_path = cv2.approxPolyDP(contour, max_error_pix, closed=False)
    line_segment_path = [p[0] for p in line_segment_path]
    if verbose: 
        print('max_path_error_m =', max_error_m)
        print('max_path_error_pix =', max_error_pix)
        print('line_segment_path =', line_segment_path)
    return line_segment_path

        
def break_path_into_segments(pix_path, max_segment_length_mm, mm_per_pix):
    # Add vertices to the path until all line segments between the
    # vertices are shorter than max_segment_length_mm. This can be
    # used to generate waypoints along the linear paths.
    #
    # The resulting path has the same form as the input path,
    # potentially with additional vertices.
    #
    # path = [[vx0, vy0], [vx1, vy1], ...]
    #
    # Not very efficient version for now. A recursive version would be
    # more efficient, but I worry about Python's recursion depth
    # restrictions, which got me in trouble before. This could also be
    # more efficient iteratively.
    max_segment_length_pix = max_segment_length_mm / mm_per_pix
    new_path = pix_path
    split_made = True
    while split_made:
        new_path, split_made = split_paths(new_path, max_segment_length_pix)
    return new_path


def split_paths(pix_path, max_segment_length_pix):
    # Not very efficient version for now. A recursive version would be
    # more efficient, but I worry about Python's recursion depth
    # restrictions, which got me in trouble before. This could also be
    # more efficient iteratively.
    split_made = False
    new_path = []
    if len(pix_path) >= 2:
        for p0, p1 in zip(pix_path, pix_path[1:]):
            new_path.append(p0)
            p0_a = np.array(p0)
            p1_a = np.array(p1)
            length = np.linalg.norm(p1_a - p0_a)
            if length > max_segment_length_pix:
                split_made = True
                p_mid = np.int64(np.round((p0_a + p1_a) / 2.0))
                new_path.append(list(p_mid))
            new_path.append(p1)
    else:
        print('WARNING: split_paths given pix_path input with length <= 1.')
        print('         returning pix_path without modification')
        print('         pix_path =', pix_path)
        return pix_path
    return new_path, split_made


def chop_path_at_location(pix_path, best_stopping_location):
    # Terminate the path at the provided
    # best_stopping_location. Attempts to find a line segment of the
    # graph that is close to the location at which to chop off the
    # rest of the path. WARNING: CURRENTLY THIS APPEARS TO HAVE A BUG
    # IN THAT A LINE SEGMENT COULD BE FAR FROM THE LOCATION BUT
    # POINTING AT IT AND END UP BEING SELECTED FOR THE CUT.
    pb_a = np.array(best_stopping_location)
    if len(pix_path) >= 2:
        min_perp_dist = None
        min_index = None
        for i, (p0, p1) in enumerate(zip(pix_path, pix_path[1:])):
            p0_a = np.array(p0)
            p1_a = np.array(p1)
            v0b = pb_a - p0_a
            v01 = p1_a - p0_a
            v01_n = v01 / np.linalg.norm(v01)
            perp_dist = np.linalg.norm(v0b - (v01_n * np.dot(v0b, v01_n)))
            if (min_perp_dist is None) or (perp_dist < min_perp_dist):
                min_perp_dist = perp_dist
                min_index = i
        new_path = pix_path[:min_index+1]
        new_path.append(best_stopping_location)
    else:
        print('WARNING: chop_path_at_location given pix_path input with length <= 1.')
        print('         returning pix_path without modification')
        print('         pix_path =', pix_path)
        return pix_path
    return new_path
    


