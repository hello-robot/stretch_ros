#!/usr/bin/env python3

import numpy as np
import scipy.ndimage as nd
import scipy.signal as si
import cv2
import math
import stretch_funmap.max_height_image as mh
import stretch_funmap.segment_max_height_image as sm
import stretch_funmap.ros_max_height_image as rm
import hello_helpers.hello_misc as hm
import ros_numpy as rn
import rospy
import os

from stretch_funmap.numba_manipulation_planning import numba_find_base_poses_that_reach_target, numba_check_that_tool_can_deploy
from stretch_funmap.numba_check_line_path import numba_find_contact_along_line_path, numba_find_line_path_on_surface

def plan_surface_coverage(tool_start_xy_pix, tool_end_xy_pix, tool_extension_direction_xy_pix, step_size_pix, max_extension_pix, surface_mask_image, obstacle_mask_image):
    # This was designed to be used when planning to clean a flat
    # surface. It could potentially be used for other purposes.
    #
    # Find the parts of linear paths that move over the surface
    # without being stopped by obstacles. This is typically used as
    # follows:
    #
    # tool_start_xy_pix and tool_end_xy_pix : Represent a line along
    # which the tool will move when the arm is retracted and the
    # mobile moves.
    #
    # tool_extension_direction_xy_pix : Direction that the tool moves
    # when the arm is extended.
    #
    # step_size_pix : The distance the mobile base moves the tool
    # along the linear path described by tool_start_xy_pix and
    # tool_end_xy_pix between arm extensions.
    #
    # max_extension_pix : The maximum distance the tool is allowed to
    # extend from the provide tool line.
    #
    # surface_mask_image : Surface that the tool is supposed to cover.
    #
    # obstacle_mask_image : Obstacles that can prevent the tool from
    # extending.
    #
    # linear_paths : Returned output consisting of the nonobstructed
    # linear extension paths that overlap the surface. [[path_number,
    # retracted_tool_position, [start_of_surface_overlap,
    # end_of_surface_overlap], ...]
    
    step_vector = tool_end_xy_pix - tool_start_xy_pix
    total_step_distance_pix = np.linalg.norm(step_vector)
    step_direction = step_vector / total_step_distance_pix
    num_steps = int(np.floor(total_step_distance_pix / step_size_pix))

    linear_paths = []

    start_xy_pix = tool_start_xy_pix
    for n in range(num_steps): 
        end_xy_pix = np.int32(np.round(start_xy_pix + (max_extension_pix * tool_extension_direction_xy_pix)))
        first_surface_contact_xy, last_surface_contact_xy, first_obstacle_contact_xy = numba_find_line_path_on_surface(np.int32(np.round(start_xy_pix)), end_xy_pix, surface_mask_image, obstacle_mask_image)
        if first_surface_contact_xy is not None:
            surface_overlap_start_xy_pix = np.array(first_surface_contact_xy)
            surface_overlap_end_xy_pix = np.array(last_surface_contact_xy)
            linear_paths.append([n, start_xy_pix.copy(), [surface_overlap_start_xy_pix, surface_overlap_end_xy_pix]])
        start_xy_pix += step_size_pix * step_direction

    return linear_paths



def detect_cliff(image, m_per_pix, m_per_height_unit, robot_xy_pix, display_text='', display_images=False):
    blur = True
    if blur: 
        blur_size = (7,7)
        image = cv2.GaussianBlur(image, blur_size, 0) 
    
    # sobel operator does not appear to be normalized to provide a true estimate of the derivative
    # 3x3 = -1 0 +1
    #       -2 0 +2
    #       -1 0 +1
    sobel_width = 3
    sobel_factor = 4.0
    initial_edge_diff_m = 0.2 / sobel_factor
    linking_edge_diff_m = 0.1 / sobel_factor
    initial_threshold = initial_edge_diff_m / m_per_height_unit
    linking_threshold = linking_edge_diff_m / m_per_height_unit
    canny_edges = cv2.Canny(image, initial_threshold, linking_threshold, apertureSize=sobel_width, L2gradient=True)

    # remove cliffs that are not large enough
    # real-world vertical cliffs could have an associated steep slope in a depth image
    min_edge_height_m = 0.2
    min_edge_height_pix = min_edge_height_m / m_per_height_unit
    canny_edges[image < min_edge_height_pix] = 0
    
    use_dilation = True 
    if use_dilation:
        kernel_width_pix = 3
        iterations = 1
        kernel_radius_pix = (kernel_width_pix - 1) // 2
        kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
        cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
        canny_edges = cv2.dilate(canny_edges, kernel, iterations=iterations)

    if display_images:
        h, w = image.shape
        color_im = np.zeros((h, w, 3), np.uint8)
        color_im[:,:,0] = image
        color_im[:,:,1] = image
        color_im[:,:,2] = image

    min_edge_length_m = 0.1
    min_gap_m = 0.1
    
    minLineLength = min_edge_length_m / m_per_pix
    print('minLineLength = {0}'.format(minLineLength))
    maxLineGap = min_gap_m / m_per_height_unit
    threshold = 50
    rho = 1
    degrees_of_resolution = 2.0
    angular_resolution = degrees_of_resolution * (np.pi/180.0)
    lines = cv2.HoughLinesP(canny_edges, rho, angular_resolution, threshold, minLineLength=minLineLength, maxLineGap=maxLineGap)
    if (lines is not None) and (display_images):
        print('Found {0} lines.'.format(len(lines)))
        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            j = i + 1
            color = [(100 * j) % 255, (100 * (j+1)) % 255, 255]
            c = np.array(color)
            c = c * (255.0/np.max(c))
            width = 1
            cv2.line(color_im, (x1,y1), (x2,y2), c, width)

    # Find the best cliff that is closest to the robot, but is not the
    # robot's own arm.
    arm_vec = [0.0, -1.0]
    min_angle_to_arm_deg = 20.0
    ang_thresh = np.cos(min_angle_to_arm_deg * (np.pi/180.0))
    robot_loc = np.array(robot_xy_pix)
    candidates = []
    if lines is not None:
        for i, line in enumerate(lines):
            x1, y1, x2, y2 = line[0]
            line_vec = np.array([x2 - x1, y2 - y1])
            line_vec = line_vec / np.linalg.norm(line_vec)
            proj = np.dot(arm_vec, line_vec)
            if abs(proj) < ang_thresh:
                line_center = np.array([(x1 + x2)/2.0, (y1 + y2)/2.0])
                dist = np.linalg.norm(line_center - robot_loc)
                candidates.append([dist, [x1, y1, x2, y2]])
        
    if len(candidates) > 0:
        # sort by straight line distance
        def sort_by_distance(cliff):
            return cliff[0]
        candidates.sort(key=sort_by_distance)
        best_candidate = candidates[0][1]
        if display_images: 
            width = 2
            x1, y1, x2, y2 = best_candidate
            cv2.line(color_im, (x1,y1), (x2,y2), [0,0,255], width)
    else:
        print('No viable cliff candidates found.')
        best_candidate = None

    if best_candidate is not None: 
        x0, y0, x1, y1 = best_candidate
        p0 = [x0, y0]
        p1 = [x1, y1]

        # find normal vector to the cliff on the robot's side of the cliff
        cliff_vec = np.array([x1 - x0, y1 - y0])
        robot_vec = np.array([robot_xy_pix[0] - x0, robot_xy_pix[1] - y0])
        cliff_vec = cliff_vec / np.linalg.norm(cliff_vec)
        robot_vec = robot_vec / np.linalg.norm(robot_vec)
        proj = np.dot(cliff_vec, robot_vec)
        normal = robot_vec - (proj * cliff_vec)
        normal = normal / np.linalg.norm(normal)

        if display_images:
            normal_length = 40.0
            x1, y1 = np.int64(np.round(normal_length * normal))
            x0 = np.int(np.round((p0[0] + p1[0])/2.0))
            y0 = np.int(np.round((p0[1] + p1[1])/2.0))
            width = 1
            cv2.line(color_im, (x0,y0), (x1+x0,y1+y0), [0,255,0], width)
    else:
        p0 = None
        p1 = None
        normal = None

    if display_images: 
        cv2.imshow('image ' + display_text, image)
        cv2.imshow('canny edges ' + display_text, canny_edges)
        cv2.imshow('depth image with detected lines ' + display_text, color_im)
        
    return p0, p1, normal
    

class ManipulationView():
    def __init__(self, tf2_buffer, debug_directory=None):
        self.debug_directory = debug_directory
        print('ManipulationView __init__: self.debug_directory =', self.debug_directory)
        
        # Define the volume of interest for planning using the current
        # view.
        
        # How far to look ahead.
        look_ahead_distance_m = 2.0
        # Robot's width plus a safety margin.
        look_to_side_distance_m = 1.3

        m_per_pix = 0.006
        pixel_dtype = np.uint8 

        # stretch (based on HeadScan in mapping.py)
        robot_head_above_ground = 1.13
        # After calibration, the floor is lower for stretch than for
        # Django, so I've lowered the acceptable floor range even
        # more. This is merits more thought. Is there something
        # wrong with the calibration or is this to be expected?
        # How consistent will it be with different floor types?
        # How will the robot handle floor slope due to calibration
        # / hardware issues?
        lowest_distance_below_ground = 0.03
        voi_height_m = robot_head_above_ground + lowest_distance_below_ground

        robot_right_edge_m = 0.2
        voi_side_x_m = 2.0 * look_to_side_distance_m
        voi_side_y_m = look_ahead_distance_m
        
        voi_axes = np.identity(3)
        voi_origin = np.array([-(voi_side_x_m/2.0), -(voi_side_y_m + robot_right_edge_m), -lowest_distance_below_ground])

        # Define the VOI using the base_link frame
        old_frame_id = 'base_link'
        voi = rm.ROSVolumeOfInterest(old_frame_id, voi_origin, voi_axes, voi_side_x_m, voi_side_y_m, voi_height_m)
        # Convert the VOI to the map frame to handle mobile base changes
        new_frame_id = 'map'
        lookup_time = rospy.Time(0) # return most recent transform
        timeout_ros = rospy.Duration(0.1)
        stamped_transform =  tf2_buffer.lookup_transform(new_frame_id, old_frame_id, lookup_time, timeout_ros)
        points_in_old_frame_to_new_frame_mat = rn.numpify(stamped_transform.transform)
        voi.change_frame(points_in_old_frame_to_new_frame_mat, new_frame_id)
        self.voi = voi
        self.max_height_im = rm.ROSMaxHeightImage(self.voi, m_per_pix, pixel_dtype)
        self.max_height_im.print_info()
        self.updated = False

    def move_head(self, move_to_pose):
        tilt = -0.8
        pan = -1.8 #-1.6
        # This head configuration can reduce seeing the hand or arm when they are held high, which can avoid noise due to the hand and arm being to close to the head.
        #tilt = -0.6
        #pan = -0.9
        pose = {'joint_head_pan': pan, 'joint_head_tilt': tilt}
        move_to_pose(pose)
        head_settle_time = 0.5
        rospy.sleep(head_settle_time)

    def estimate_reach_to_contact_distance(self, tooltip_frame, tf2_buffer, save_debugging_images=True): 
        h = self.max_height_im
        m_per_pix = h.m_per_pix
        tooltip_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(tooltip_frame, tf2_buffer)
        # Obtain the tooltip location in the image by obtaining the
        # translational component of the transform, which is the same
        # as multiplying by [0,0,0,1]
        tooltip_x, tooltip_y, tooltip_z = tooltip_points_to_image_mat[:, 3][:3]
        
        base_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat('base_link', tf2_buffer)
        # Ideal arm extension direction is in the negative y axis
        # direction of the base_link frame. This could be improved by
        # using the calibrated URDF to find the Jacobian for
        # extension.
        #
        # extension_direction = np.array([0.0, -1.0, 0.0])
        #
        # select the first 2 components of the y axis and negate them
        extension_xy = -base_points_to_image_mat[:, 1][:2]
        # create a unit length vector in the direction of extension in the image
        extension_xy = extension_xy / np.linalg.norm(extension_xy)
        
        start_xy = np.array([tooltip_x, tooltip_y])
        max_reach = 0.5 / m_per_pix
        end_xy = (max_reach * extension_xy) + start_xy
        # Ensure that the arm and hand are not perceived as obstacles
        # by moving the start location a little in front of the
        # tooltip.
        min_obstacle_distance = 0.02 / m_per_pix # 2 cm in front of the gripper
        start_xy = start_xy + (min_obstacle_distance * extension_xy)
        # WARNING: THIS WILL NOT WORK FOR COVERED AREAS, SINCE IT USES THE MAXIMUM HEIGHT OF A REGION
        safety_distance_pix = 2

        # Avoid obstacles that are a little under or greater than the
        # height of the tool.
        mask_image = 255 * np.uint8(h.image > (tooltip_z - safety_distance_pix))

        # Dilate the obstacles to create a safety margin.
        use_dilation = False
        if use_dilation:
            kernel_width_pix = 3
            iterations = 1
            kernel_radius_pix = (kernel_width_pix - 1) // 2
            kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
            cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
            mask_image = cv2.dilate(mask_image, kernel, iterations=iterations)

        rospy.loginfo('*************************************')
        rospy.loginfo('start_xy = {0}'.format(start_xy))
        rospy.loginfo('end_xy = {0}'.format(end_xy))
        rospy.loginfo('*************************************')
            
        contact_found, (contact_x, contact_y) = numba_find_contact_along_line_path(start_xy, end_xy, mask_image)
        if contact_found:
            print('ManipulationView estimate_reach_to_contact_distance : contact detected!')
            contact_xy = np.array([contact_x, contact_y])
            reach_m = np.linalg.norm(contact_xy - start_xy) * m_per_pix
        else:
            print('ManipulationView estimate_reach_to_contact_distance : WARNING - NO CONTACT DETECTED')
            reach_m = None

        print('ManipulationView estimate_reach_to_contact_distance : self.debug_directory =', self.debug_directory)
        print('ManipulationView estimate_reach_to_contact_distance : save_debugging_image =', save_debugging_images)
        if save_debugging_images and (self.debug_directory is not None):
            dirname = self.debug_directory + 'estimate_reach_to_contact_distance/'
            # If the directory does not already exist, create it.
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            filename = 'estimate_reach_to_contact_distance_mask_' + hm.create_time_string() + '.png'
            cv2.imwrite(dirname + filename, mask_image)

            line_width = 2
            radius = 5
            p0 = tuple(np.int32(np.round(start_xy)))
            
            height, width = mask_image.shape
            color_im = np.zeros((height, width, 3), np.uint8)
            color_im[:,:,0] = mask_image
            color_im[:,:,1] = mask_image
            color_im[:,:,2] = mask_image
            # always draw the start point, regardless of contact detection
            cv2.circle(color_im, p0, radius, (0,255,0), 1)
            if contact_found:                
                p1 = tuple(np.int32(np.round(contact_xy)))
                cv2.line(color_im, p0, p1, [255, 0, 0], line_width)
                cv2.circle(color_im, p1, radius, (0,0,255), 1)
            filename = 'estimate_reach_to_contact_distance_annotated_mask_' + hm.create_time_string() + '.png'
            cv2.imwrite(dirname + filename, color_im)
            
            rgb_image = self.max_height_im.rgb_image.copy()
            # always draw the start point, regardless of contact detection
            cv2.circle(rgb_image, p0, radius, (0,255,0), 1)
            if contact_found:
                p1 = tuple(np.int32(np.round(contact_xy)))
                cv2.line(rgb_image, p0, p1, [255, 0, 0], line_width)
                cv2.circle(rgb_image, p1, radius, (0,0,255), 1)
            filename = 'estimate_reach_to_contact_distance_annotated_rgb_' + hm.create_time_string() + '.png'
            cv2.imwrite(dirname + filename, rgb_image)

        else:
            rospy.loginfo('ManipulationView estimate_reach_to_contact_distance: No debug directory provided, so debugging data will not be saved.')

            
        return reach_m

    def get_grasp_target(self, tf2_buffer, max_object_planar_distance_m=1.0):
        grasp_target = sm.find_object_to_grasp(self.max_height_im, display_on=False)
        if grasp_target is None:
            return None
        
        h = self.max_height_im
        m_per_pix = h.m_per_pix
        
        debug = True
        if debug and (self.debug_directory is not None): 
            rgb_image = h.rgb_image.copy()
            sm.draw_grasp(rgb_image, grasp_target)
            # Save the new scan to disk.
            dirname = self.debug_directory + 'get_grasp_target/'
            filename = 'grasp_target_' + hm.create_time_string() + '.png'
            print('ManipulationView get_grasp_target : directory =', dirname)
            print('ManipulationView get_grasp_target : filename =', filename)
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            cv2.imwrite(dirname + filename, rgb_image)
        base_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat('base_link', tf2_buffer)
        base_xy_pix = base_points_to_image_mat[:, 3][:2]
        grasp_xy_pix = grasp_target['location_xy_pix']
        object_planar_distance_m = m_per_pix * np.linalg.norm(base_xy_pix - grasp_xy_pix)
        print('object_planar_distance_m =', object_planar_distance_m)
        if object_planar_distance_m >= max_object_planar_distance_m:
            return None
            
        return grasp_target

    
    def get_pregrasp_lift(self, grasp_target, tf2_buffer):
        h = self.max_height_im
        m_per_unit = h.m_per_height_unit
        tooltip_frame = 'link_grasp_center'
        tooltip_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(tooltip_frame, tf2_buffer)
        
        # Obtain the tooltip location in the image by obtaining the
        # translational component of the transform, which is the same
        # as multiplying by [0,0,0,1]
        tooltip_x, tooltip_y, tooltip_z = tooltip_points_to_image_mat[:, 3][:3]
        tool_current_xy_pix = np.array([tooltip_x, tooltip_y])

        lift_to_pregrasp_m = m_per_unit * (grasp_target['location_z_pix'] - tooltip_z)
        # Ensure it is above the surface
        extra_pregrasp_height_m = 0.1
        lift_to_pregrasp_m = lift_to_pregrasp_m + extra_pregrasp_height_m
        if lift_to_pregrasp_m > 0.94:
            lift_to_pregrasp_m = 0.94
        return lift_to_pregrasp_m

    
    def get_pregrasp_yaw(self, grasp_target, tf2_buffer):
        h = self.max_height_im
        # The planar component of the link gripper x_axis is parallel
        # to the middle of the gripper, but points in the opposite
        # direction.
        gripper_frame = 'link_gripper'
        gripper_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(gripper_frame, tf2_buffer)
        #
        # forward_direction = np.array([1.0, 0.0, 0.0])
        #
        # select the first 2 components of the x axis.
        forward_xy = gripper_points_to_image_mat[:, 0][:2]
        # create a unit length vector in the direction of extension in the image
        gripper_forward_pix = forward_xy / np.linalg.norm(forward_xy)
        
        elongated_object = grasp_target['elongated']

        if not elongated_object:
            # Point the gripper straight out to grasp the object,
            # since the gripper's orientation is not expected to
            # matter.
            yaw_angle = 0.0
        else:
            gripper_ang_rad = np.arctan2(gripper_forward_pix[1], -gripper_forward_pix[0])
            
            centroid = np.array(grasp_target['location_xy_pix'])
            long_axis = np.array(grasp_target['long_axis_pix'])
            
            v0 = long_axis[0] - centroid
            v0 = v0 / np.linalg.norm(v0)
            d0 = np.dot(v0, gripper_forward_pix)
            
            v1 = long_axis[1] - centroid
            v1 = v1 / np.linalg.norm(v1)
            d1 = np.dot(v1, gripper_forward_pix)

            if d0 > d1: 
                side_to_grasp = v0
            else:
                side_to_grasp = v1
                
            object_ang_rad = np.arctan2(side_to_grasp[1], -side_to_grasp[0])            
            yaw_angle = float(hm.angle_diff_rad(object_ang_rad, gripper_ang_rad))

        return yaw_angle

    
    def get_pregrasp_planar_translation(self, grasp_target, tf2_buffer):
        h = self.max_height_im
        m_per_pix = h.m_per_pix
        
        # The planar component of the link gripper x_axis is parallel
        # to the middle of the gripper, but points in the opposite
        # direction.
        gripper_frame = 'link_gripper'
        gripper_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(gripper_frame, tf2_buffer)

        # Obtain the gripper yaw axis location in the image by
        # obtaining the translational component of the transform,
        # which is the same as multiplying by [0,0,0,1]
        yaw_x, yaw_y, yaw_z = gripper_points_to_image_mat[:, 3][:3]
        yaw_xy_pix = np.array([yaw_x, yaw_y])
        #
        # forward_direction = np.array([1.0, 0.0, 0.0])
        #
        # select the first 2 components of the x axis.
        forward_xy = gripper_points_to_image_mat[:, 0][:2]
        gripper_forward_pix = forward_xy / np.linalg.norm(forward_xy)

        base_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat('base_link', tf2_buffer)
        # Ideal arm extension direction is in the negative y axis
        # direction of the base_link frame. This could be improved by
        # using the calibrated URDF to find the Jacobian for
        # extension.
        #
        # extension_direction = np.array([0.0, -1.0, 0.0])
        #
        # select the first 2 components of the y axis and negate them
        extension_xy = -base_points_to_image_mat[:, 1][:2]
        # create a unit length vector in the direction of extension in the image
        tool_extension_direction_xy_pix = extension_xy / np.linalg.norm(extension_xy)
        # Ideal base forward motion direction is in the positive x axis direction of the base_link frame.
        #
        # forward_direction = np.array([1.0, 0.0, 0.0])
        #
        # select the first 2 components of the x axis.
        forward_xy = base_points_to_image_mat[:, 0][:2]
        # create a unit length vector in the direction of extension in the image
        robot_forward_pix = forward_xy / np.linalg.norm(forward_xy)
        #robot_forward_pix = np.array([np.cos(robot_ang_rad), -np.sin(robot_ang_rad)])

        # target distance from yaw joint axis to the object grasp
        # location
        pregrasp_target_dist_m = 0.27
        pregrasp_target_dist_pix = pregrasp_target_dist_m / m_per_pix
        pregrasp_target_xy_pix = (pregrasp_target_dist_pix * gripper_forward_pix) + grasp_target['location_xy_pix']
        
        translate_xy_pix = pregrasp_target_xy_pix - yaw_xy_pix
        robot_forward_m = m_per_pix * np.dot(translate_xy_pix, robot_forward_pix)
        wrist_extension_m = m_per_pix * np.dot(translate_xy_pix, tool_extension_direction_xy_pix)

        debug = True
        if debug and (self.debug_directory is not None): 
            rgb_image = h.rgb_image.copy()
            radius = 5
            width = 1
            line_width = 1
            line_length = 10.0
            
            cv2.circle(rgb_image, tuple(np.int32(np.round(grasp_target['location_xy_pix']))), radius, [0, 0, 255], width)
            cv2.circle(rgb_image, tuple(np.int32(np.round(pregrasp_target_xy_pix))), radius, [0, 255, 0], width)
            cv2.circle(rgb_image, tuple(np.int32(np.round(yaw_xy_pix))), radius, [255, 0, 0], width)

            x0 = np.int32(np.round(yaw_xy_pix))
            x1 = np.int32(np.round((line_length * np.array(tool_extension_direction_xy_pix)) + np.array(yaw_xy_pix)))
            cv2.line(rgb_image, tuple(x0), tuple(x1), [255, 255, 255], line_width)
            
            x0 = np.int32(np.round(yaw_xy_pix))
            x1 = np.int32(np.round((line_length * np.array(robot_forward_pix)) + np.array(yaw_xy_pix)))
            cv2.line(rgb_image, tuple(x0), tuple(x1), [255, 255, 255], line_width)
            
            x0 = np.int32(np.round(yaw_xy_pix))
            x1 = np.int32(np.round((line_length * np.array(gripper_forward_pix)) + np.array(yaw_xy_pix)))
            cv2.line(rgb_image, tuple(x0), tuple(x1), [0, 255, 255], line_width)

            centroid = grasp_target['location_xy_pix']
            x0 = np.int32(np.round(centroid))
            x1 = np.int32(np.round((pregrasp_target_dist_pix * np.array(gripper_forward_pix)) + np.array(centroid)))
            cv2.line(rgb_image, tuple(x0), tuple(x1), [255, 255, 0], line_width)

            # Save the new scan to disk.
            dirname = self.debug_directory + 'get_pregrasp_planar_translation/'
            filename = 'pregrasp_planar_translation_' + hm.create_time_string() + '.png'
            print('ManipulationView get_pregrasp_planar_translation : directory =', dirname)
            print('ManipulationView get_pregrasp_planar_translation : filename =', filename)
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            cv2.imwrite(dirname + filename, rgb_image)
            
        pregrasp_mobile_base_m = robot_forward_m
        pregrasp_wrist_extension_m = wrist_extension_m
        
        return pregrasp_mobile_base_m, pregrasp_wrist_extension_m

    
    def get_grasp_from_pregrasp(self, grasp_target, tf2_buffer):

        h = self.max_height_im
        m_per_unit = h.m_per_height_unit
        m_per_pix = h.m_per_pix
        
        fingertip_frame = 'link_gripper_fingertip_left'
        fingertip_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(fingertip_frame, tf2_buffer)
        
        # Obtain the fingertip location in the image by obtaining the
        # translational component of the transform, which is the same
        # as multiplying by [0,0,0,1]
        fingertip_x, fingertip_y, fingertip_z = fingertip_points_to_image_mat[:, 3][:3]
        fingertip_xy_pix = np.array([fingertip_x, fingertip_y])

        grasp_lift_m = m_per_unit * (grasp_target['location_z_pix'] - fingertip_z)
        # lower to account for compliant fingers and finger raising when closing
        grasp_lift_m = grasp_lift_m + 0.01
        
        # The planar component of the link gripper x_axis is parallel
        # to the middle of the gripper, but points in the opposite
        # direction.
        gripper_frame = 'link_gripper'
        gripper_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(gripper_frame, tf2_buffer)

        # Obtain the gripper yaw axis location in the image by
        # obtaining the translational component of the transform,
        # which is the same as multiplying by [0,0,0,1]
        yaw_x, yaw_y, yaw_z = gripper_points_to_image_mat[:, 3][:3]
        yaw_xy_pix = np.array([yaw_x, yaw_y])
        #
        # forward_direction = np.array([1.0, 0.0, 0.0])
        #
        # select the first 2 components of the x axis.
        forward_xy = gripper_points_to_image_mat[:, 0][:2]
        gripper_forward_pix = forward_xy / np.linalg.norm(forward_xy)

        base_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat('base_link', tf2_buffer)
        # Ideal arm extension direction is in the negative y axis
        # direction of the base_link frame. This could be improved by
        # using the calibrated URDF to find the Jacobian for
        # extension.
        #
        # extension_direction = np.array([0.0, -1.0, 0.0])
        #
        # select the first 2 components of the y axis and negate them
        extension_xy = -base_points_to_image_mat[:, 1][:2]
        # create a unit length vector in the direction of extension in the image
        tool_extension_direction_xy_pix = extension_xy / np.linalg.norm(extension_xy)
        # Ideal base forward motion direction is in the positive x axis direction of the base_link frame.
        #
        # forward_direction = np.array([1.0, 0.0, 0.0])
        #
        # select the first 2 components of the x axis.
        forward_xy = base_points_to_image_mat[:, 0][:2]
        # create a unit length vector in the direction of extension in the image
        robot_forward_pix = forward_xy / np.linalg.norm(forward_xy)

        # target distance from yaw joint axis to the object grasp
        # location
        grasp_target_dist_m = 0.21
        grasp_target_dist_pix = grasp_target_dist_m / m_per_pix
        grasp_target_xy_pix = (grasp_target_dist_pix * gripper_forward_pix) + grasp_target['location_xy_pix']
        
        translate_xy_pix = grasp_target_xy_pix - yaw_xy_pix
        robot_forward_m = m_per_pix * np.dot(translate_xy_pix, robot_forward_pix)
        wrist_extension_m = m_per_pix * np.dot(translate_xy_pix, tool_extension_direction_xy_pix)

        debug = True
        if debug and (self.debug_directory is not None): 
            rgb_image = h.rgb_image.copy()
            radius = 5
            width = 1
            line_width = 1
            line_length = 10.0
            
            cv2.circle(rgb_image, tuple(np.int32(np.round(grasp_target['location_xy_pix']))), radius, [0, 0, 255], width)
            cv2.circle(rgb_image, tuple(np.int32(np.round(grasp_target_xy_pix))), radius, [0, 255, 0], width)
            cv2.circle(rgb_image, tuple(np.int32(np.round(yaw_xy_pix))), radius, [255, 0, 0], width)

            x0 = np.int32(np.round(yaw_xy_pix))
            x1 = np.int32(np.round((line_length * np.array(tool_extension_direction_xy_pix)) + np.array(yaw_xy_pix)))
            cv2.line(rgb_image, tuple(x0), tuple(x1), [255, 255, 255], line_width)
            
            x0 = np.int32(np.round(yaw_xy_pix))
            x1 = np.int32(np.round((line_length * np.array(robot_forward_pix)) + np.array(yaw_xy_pix)))
            cv2.line(rgb_image, tuple(x0), tuple(x1), [255, 255, 255], line_width)

            x0 = np.int32(np.round(yaw_xy_pix))
            x1 = np.int32(np.round((line_length * np.array(gripper_forward_pix)) + np.array(yaw_xy_pix)))
            cv2.line(rgb_image, tuple(x0), tuple(x1), [0, 255, 255], line_width)

            centroid = grasp_target['location_xy_pix']
            x0 = np.int32(np.round(centroid))
            x1 = np.int32(np.round((grasp_target_dist_pix * np.array(gripper_forward_pix)) + np.array(centroid)))
            cv2.line(rgb_image, tuple(x0), tuple(x1), [255, 255, 0], line_width)

            # Save the new scan to disk.
            dirname = self.debug_directory + 'get_grasp_from_pregrasp/'
            filename = 'grasp_from_pregrasp_' + hm.create_time_string() + '.png'
            print('ManipulationView get_grasp_from_pregrasp : directory =', dirname)
            print('ManipulationView get_grasp_from_pregrasp : filename =', filename)
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            cv2.imwrite(dirname + filename, rgb_image)
  
        grasp_mobile_base_m = robot_forward_m
        grasp_wrist_extension_m = wrist_extension_m
        
        return grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m
    
    
    def get_surface_wiping_plan(self, tf2_buffer, tool_width_m, tool_length_m, step_size_m):
        strokes = None
        movements = None
        surface_height_m = None
        if self.updated:
            h = self.max_height_im
            h_image = h.image
            m_per_unit = h.m_per_height_unit
            m_per_pix = h.m_per_pix

            tool_width_pix = tool_width_m / m_per_pix
            tool_length_pix = tool_length_m / m_per_pix
            
            wrist_frame = 'link_aruco_top_wrist'
            wrist_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(wrist_frame, tf2_buffer)
            
            # Obtain the wrist location in the image by obtaining the
            # translational component of the transform, which is the same
            # as multiplying by [0,0,0,1]
            wrist_x, wrist_y, wrist_z = wrist_points_to_image_mat[:, 3][:3]
            wrist_current_xy_pix = np.array([wrist_x, wrist_y])
            
            # Find the flat surface closest to the wrist.
            surface_mask, plane_parameters = sm.find_closest_flat_surface(h, wrist_current_xy_pix, display_on=False)

            if surface_mask is not None:
                
                # Use the maximum height on the segmented surface as a
                # conservative height for the plane.
                surface_height_pix = np.max(h_image[surface_mask > 0])
                surface_height_m = m_per_pix * surface_height_pix
                h.apply_planar_correction(plane_parameters, surface_height_pix)

                # Detect obstacles on the plane
                min_obstacle_height_m = surface_height_m + 0.015
                min_obstacle_height_pix = min_obstacle_height_m / m_per_unit
                obstacle_selector = h_image > min_obstacle_height_pix
                obstacle_mask = np.uint8(obstacle_selector)
                
                # Dilate the obstacles to create a safety margin.
                dilate_obstacles = True
                if dilate_obstacles:
                    kernel_radius_pix = int(round(max(tool_width_pix, tool_length_pix)/2.0))
                    kernel_width_pix = 1 + (2 * kernel_radius_pix)
                    iterations = 1
                    kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
                    cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
                    obstacle_mask = cv2.dilate(obstacle_mask, kernel, iterations=iterations)

                # Erode the surface to create a safety margin.
                erode_surface = True
                if erode_surface:
                    kernel_radius_pix = int(round(max(tool_width_pix, tool_length_pix)/2.0))
                    kernel_width_pix = 1 + (2 * kernel_radius_pix)
                    iterations = 1
                    kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
                    cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
                    surface_mask = cv2.erode(surface_mask, kernel, iterations=iterations)

                # make a surface cleaning plan from the right to the
                # left side of the surface
                base_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat('base_link', tf2_buffer)
                
                # Ideal arm extension direction is in the negative y axis
                # direction of the base_link frame. This could be improved by
                # using the calibrated URDF to find the Jacobian for
                # extension.
                #
                # extension_direction = np.array([0.0, -1.0, 0.0])
                #
                # select the first 2 components of the y axis and negate them
                extension_xy = -base_points_to_image_mat[:, 1][:2]
                # create a unit length vector in the direction of extension in the image
                tool_extension_direction_xy_pix = extension_xy / np.linalg.norm(extension_xy)

                # Ideal base forward motion direction is in the positive x axis direction of the base_link frame.
                #
                # forward_direction = np.array([1.0, 0.0, 0.0])
                #
                # select the first 2 components of the x axis.
                forward_xy = base_points_to_image_mat[:, 0][:2]
                # create a unit length vector in the direction of extension in the image
                robot_forward_pix = forward_xy / np.linalg.norm(forward_xy)
                
                max_drive_forward_m = 0.4 #0.25
                max_drive_backward_m = 0.4 #0.25
                max_drive_forward_pix = max_drive_forward_m / m_per_pix
                max_drive_backward_pix = max_drive_backward_m / m_per_pix

                tooltip_frame = 'link_grasp_center'
                tooltip_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(tooltip_frame, tf2_buffer)
                
                # Obtain the tooltip location in the image by obtaining the
                # translational component of the transform, which is the same
                # as multiplying by [0,0,0,1]
                tooltip_x, tooltip_y, tooltip_z = tooltip_points_to_image_mat[:, 3][:3]
                tool_current_xy_pix = np.array([tooltip_x, tooltip_y])
            
                tool_start_xy_pix = tool_current_xy_pix - (max_drive_backward_pix * robot_forward_pix)
                tool_end_xy_pix = tool_current_xy_pix + (max_drive_forward_pix * robot_forward_pix)

                step_size_pix = step_size_m / m_per_pix
                max_extension_m = 0.5
                max_extension_pix = max_extension_m / m_per_pix

                strokes = plan_surface_coverage(tool_start_xy_pix, tool_end_xy_pix, tool_extension_direction_xy_pix, step_size_pix, max_extension_pix, surface_mask, obstacle_mask)

                simple_plan = []
                # compute mobile base translation and wrist extension to strokes
                previous_forward_from_start_m = 0.0
                for n, b, [s, e] in strokes:
                    drive_pix = b - tool_current_xy_pix
                    forward_from_start_m = m_per_pix * np.dot(drive_pix, robot_forward_pix)
                    mobile_base_forward_m = forward_from_start_m - previous_forward_from_start_m
                    previous_forward_from_start_m = forward_from_start_m
                    start_wrist_extension_m = m_per_pix * np.linalg.norm(s - b)
                    end_wrist_extension_m = m_per_pix * np.linalg.norm(e - b)
                    simple_plan.append({'mobile_base_forward_m': mobile_base_forward_m,
                                        'start_wrist_extension_m': start_wrist_extension_m,
                                        'end_wrist_extension_m': end_wrist_extension_m})

                lift_to_surface_m = m_per_unit * (surface_height_pix - tooltip_z)
                    
                debug = True
                if debug and (self.debug_directory is not None):
                    rgb_image = h.rgb_image.copy()
                    line_width = 1
                    for n, b, [s, e] in strokes:
                        cv2.line(rgb_image, tuple(np.int32(np.round(s))), tuple(np.int32(np.round(e))), [255, 0, 0], line_width)
                    if len(strokes) > 0:
                        first_unextended_target = strokes[0][1]
                        radius = 5
                        width = 2
                        cv2.circle(rgb_image, tuple(np.int32(np.round(first_unextended_target))), radius, [0, 255, 0], width)

                        first_surface_target = strokes[0][2][0]
                        radius = 5
                        width = 2
                        cv2.circle(rgb_image, tuple(first_surface_target), radius, [255, 0, 0], width)

                    # Save the new scan to disk.
                    dirname = self.debug_directory + 'get_surface_wiping_plan/'
                    filename = 'surface_wiping_plan_' + hm.create_time_string() + '.png'
                    print('ManipulationView get_surface_wiping_plan : directory =', dirname)
                    print('ManipulationView get_surface_wiping_plan : filename =', filename)
                    if not os.path.exists(dirname):
                        os.makedirs(dirname)
                    cv2.imwrite(dirname + filename, rgb_image)
            else:
                rospy.loginfo('No elevated surface found.')
            
        return strokes, simple_plan, lift_to_surface_m

    
    def get_nearest_cliff(self, frame_id, tf2_buffer):
        p0 = None
        p1 = None
        normal = None
        if self.updated:
            h = self.max_height_im

            robot_xy_pix, robot_ang_rad, timestamp = h.get_robot_pose_in_image(tf2_buffer)

            wrist_frame = 'link_aruco_top_wrist'
            wrist_points_to_image_mat, ip_timestamp = h.get_points_to_image_mat(wrist_frame, tf2_buffer)
            
            # Obtain the wrist location in the image by obtaining the
            # translational component of the transform, which is the same
            # as multiplying by [0,0,0,1]
            wrist_x, wrist_y, wrist_z = wrist_points_to_image_mat[:, 3][:3]
            wrist_xy_pix = np.array([wrist_x, wrist_y])
                        
            p0, p1, normal = detect_cliff(h.image, h.m_per_pix, h.m_per_height_unit, wrist_xy_pix)
            if normal is not None: 
                image_to_points_mat, ip_timestamp = h.get_image_to_points_mat(frame_id, tf2_buffer)
                p0 = np.array([p0[0], p0[1], 0.0, 1.0])
                p0 = np.matmul(image_to_points_mat, p0)[:2]
                p1 = np.array([p1[0], p1[1], 0.0, 1.0])
                p1 = np.matmul(image_to_points_mat, p1)[:2]
                normal = np.array([normal[0], normal[1], 0.0, 0.0])
                normal = np.matmul(image_to_points_mat, normal)[:2]
                
        return p0, p1, normal
        
    def update(self, point_cloud_msg, tf2_buffer):
        self.max_height_im.clear()
        cloud_time = point_cloud_msg.header.stamp
        cloud_frame = point_cloud_msg.header.frame_id
        point_cloud = rn.numpify(point_cloud_msg)
        only_xyz = False
        if only_xyz:
            xyz = rn.point_cloud2.get_xyz_points(point_cloud)
            self.max_height_im.from_points_with_tf2(xyz, cloud_frame, tf2_buffer)
        else: 
            rgb_points = rn.point_cloud2.split_rgb_field(point_cloud)
            self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, tf2_buffer)
        obstacle_im = self.max_height_im.image == 0
        self.updated = True

    def save_scan(self, filename):
        # Save the new scan to disk.
        self.max_height_im.save(filename)

    def publish_visualizations(self, voi_marker_pub, point_cloud_pub):
        marker = self.voi.get_ros_marker(duration=1000.0)
        voi_marker_pub.publish(marker)
        point_cloud = self.max_height_im.to_point_cloud()
        point_cloud_pub.publish(point_cloud)
        


class PlanarRobotModel:
    def __init__(self):        
        ##################################################################
        # PLANAR MODEL OF THE ROBOT

        # NOTE: see navigation_planning.py for related
        # values. Eventually, these should be unified into a yaml
        # file, potentially with some values determined by the
        # calibrated URDF.

        # The heights should be measured when the arm is fully extended
        # and raised. Ideally, there would also be a payload model that
        # considers deflection due to a payload.

        # -----
        # GRIPPER AND ARM
        #
        # This is a model of the gripper and arm extending. It is for when
        # the gripper is rotated so that it is almost straight out. The
        # gripper should be rotated a little toward the back of the robot,
        # so that the gripper fingers are within the width of the wrist
        # while the arm is being extended. This should also bring the
        # farthest point of the gripper close to the center of the wrist
        # width, which enables it to fit better within a circular
        # collision model. In general, this gripper pose should reduce the
        # swept volume while the arm extends.

        # 0.15 : distance from the side of the most proximal moving cuff to the
        # side of the gripper when closed and extended straight out

        # 0.14 : distance from the side of the wrist yaw cylinder to the
        # side of the most proximal moving cuff
        self.gripper_and_arm_width_m = 0.14 # USED FOR PLANNING: 14cm reasonable for Guthrie 1
        self.diameter_of_yaw_cylinder_m = 0.045
        #self.radius_of_yaw_cylinder_m = 0.0225 # USED FOR PLANNING
        self.radius_of_yaw_cylinder_m = 0.022 # USED FOR PLANNING: 2.2cm using calibers for Guthrie 1
        # safety margins for the arm extension
        self.gripper_and_arm_width_safety_margin_m = 0.01 # USED FOR PLANNING
        # distance forward from the center of the wrist yaw cylinder to
        # the center of the fingertips
        self.yaw_to_fingertips_m = 0.22 # USED FOR PLANNING: 22cm for Guthrie 1
        
        # -----
        # GRIPPER
        #
        # planar length from the center of the wrist's yaw axis to the
        # gripper's fingers when about halfway closed
        self.gripper_length_m = 0.26
        # maximum width of the gripper fingers at the wrist, which is
        # twice the distance from the edge of the servo holder to the
        # center of the yaw axis cylinder (note that this width is with
        # respect to the yaw axis cylinder and hence doesn't represent the
        # assymetry due to the servo being on one side of the gripper
        # thereby increasing the width substantially)
        self.max_gripper_width_at_wrist_m = 0.1 
        # maximum gripper width along the fingers when it is closed so the
        # fingertips are just touching each other, the measurement is made
        # where the metal bows out
        self.max_gripper_width_at_fingers_m = 0.075
        # distance from the ground to the bottom of the most proximal part
        # of the gripper at the wrist yaw joint (i.e., where the actuator
        # is) when the arm is raised and extended without a payload
        self.max_gripper_height_at_wrist_m = 1.015 #1.03 when
        # retracted distance from the ground to the bottom of the
        # gripper's fingertips when the arm is fully raised and fully
        # extended without a payload
        #self.max_gripper_height_at_fingers_m = 0.9 # USED FOR PLANNING
        self.max_gripper_height_at_fingers_m = 1.09 # USED FOR PLANNING: 1.09 with tape measure for Guthrie 1 (safety margin for other robots? what if the arm or mast are tilted?
        
        # distance from the ground to the bottom of the gripper's
        # fingertips when the arm is lowered and extended without a
        # payload
        self.min_gripper_height_at_fingers_m = 0.0


        # -----
        # ARM
        #
        # distance from the outer edge of the yaw axis cylinder to the
        # edge of the most proximal moving arm cuff
        self.max_arm_width_m = 0.14
        # distance from the ground to the bottom of the yaw axis cylinder
        self.max_arm_height_m = 1.08
        # measured from the exterior of the most proximal cuff to the
        # interior of the wrist cuff when fully extended
        # Ella: actually measured 0.51, but want to be conservative
        self.max_arm_travel_m = 0.5 # USED FOR PLANNING: about 51.25cm with Guthrie 1 (so using 0.5 for safety)
        # the height of the arm above the ground when the lift is at 0.0
                
        # -----
        # MOBILE BASE
        #
        # distance from the center of the laser range finder to the outer
        # edge of the wrist when retracted
        self.min_mobile_base_radius_m = 0.21
        # distance from the center of the laser range finder to the back
        # of the robot (does not include cables for tethering)
        self.max_mobile_base_radius_m = 0.27
        # radius of the circumscribing circle of the mobile base,
        # currently defined by the distance to the right corner and the
        # back center of the robot (does not include cables for tethering)
        self.mobile_base_circumscribed_radius_m = 0.21
        # mobile base origin with respect to the yaw axis of the fully
        # retracted arm
        #self.yaw_axis_to_origin_length_m = 0.035 # USED FOR PLANNING
        self.yaw_axis_to_origin_length_m = 0.025 # USED FOR PLANNING 2.5cm with hacky measurements on Guthrie 1 (ask Blaine to look on CAD model)
        #self.yaw_axis_to_origin_left_m = 0.0165 # USED FOR PLANNING
        self.yaw_axis_to_origin_left_m = 0.015 # USED FOR PLANNING: 15cm using tape measure with Guthrie 1
        # mobile base origin with respect to the center of the circle that
        # circumscribes the mobile base
        self.circumscribed_to_origin_m = 0.072

        ##################################################################
    

class ManipulationPlanner:
    def __init__(self):
        self.planar_model = PlanarRobotModel()
        # Region around the target over which collisions are ignored
        self.target_safe_radius_m = 0.1 # ignore 10cm radius around the target when reaching
        
    def base_pose(self, max_height_image, target_xyz_pix, robot_xya_pix, image_display_on=False):
        
        robot_xy_pix = np.int64(np.round(robot_xya_pix[:2]))
        robot_ang_rad = robot_xya_pix[2]
        
        robot_x_pix, robot_y_pix = robot_xy_pix
        target_x, target_y, target_z = target_xyz_pix
        
        image = max_height_image.image
        m_per_height_unit = max_height_image.m_per_height_unit
        m_per_pix = max_height_image.m_per_pix
        pix_per_m = 1.0 / m_per_pix
        
        # The maximum height of the bottoms of the fingers at full
        # extension. This should represent the worst case for the fingers
        # moving above objects without collisions.
        max_finger_height_m = self.planar_model.max_gripper_height_at_fingers_m
        max_finger_height_pix = max_finger_height_m / m_per_height_unit

        target_z_m = target_z * m_per_height_unit
        if target_z_m > self.planar_model.max_gripper_height_at_fingers_m:
            print('Target is too high for the fingertips to reach, so planning to reach as high as possible.')
            target_z_m = self.planar_model.max_gripper_height_at_fingers_m
        target_z_pix = target_z_m / m_per_height_unit

        # Anything taller than the target height will be considered an
        # obstacle. If this fails to find a solution, then a plan that
        # moves slightly above the target and then descends at the end
        # could be tried.
        finger_obstacle_image = np.zeros_like(image)
        finger_obstacle_image[image > target_z_pix] = 255
        
        # Remove obstacles over a small area surrounding the target. For
        # example, the target might be a switch on a wall or a tall can
        # with which contact is allowed. The target location may not be
        # outside the perceived extent of the target object.
        target_safe_radius_pix = int(round(pix_per_m * self.target_safe_radius_m))
        cv2.circle(finger_obstacle_image, (target_x, target_y), target_safe_radius_pix, 0, -1)

        h, w = image.shape

        # Estimate where the robot can navigate given its current pose
        # and and the map.
        distance_map, traversable_mask = sm.process_max_height_image(max_height_image, robot_x_pix, robot_y_pix, robot_ang_rad, display_on = False)

        # Dilate finger obstacles to account for the gripper and arm
        # widths. This should also reduce issues due to undersampling by
        # the radial search.
        reach_width_m = self.planar_model.gripper_and_arm_width_m + (2.0 * self.planar_model.gripper_and_arm_width_safety_margin_m)
        reach_width_pix = pix_per_m * reach_width_m
        # distance to dilate objects
        print('reach_width_pix =', reach_width_pix)
        reach_half_width_pix = int(round(reach_width_pix / 2.0))
        kernel_width_pix = 1 + (2 * reach_half_width_pix)
        print('kernel_width = ', kernel_width_pix)
        kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
        cv2.circle(kernel, (reach_half_width_pix, reach_half_width_pix), reach_half_width_pix, 255, -1)
        dilated_finger_obstacle_image = cv2.dilate(finger_obstacle_image, kernel)

        # Find base poses that the model predicts will be able reach
        # the target.
        
        # Should be greater than the distance from the mobile base
        # center to the fingertips when the arm is extended and the
        # wrist is out.
        approx_max_ray_m = 1.0
        circumference_pix = (2.0 * np.pi) * approx_max_ray_m * pix_per_m
        # Ensure it will not skip pixels when rotating through their
        # diagonals. This should result in a solid wring when
        # visualized using show_all_rays.
        num_angles = np.sqrt(2.0) * circumference_pix
        # This num_angles would likely work with a true star burst
        # pattern. However, the actual pattern has rays that are
        # slightly rotated. The following slop factor should be
        # manually tuned while visualizing all the rays to ensure that
        # there are no gaps.
        num_angles = 1.08 * num_angles # resulted in 1600 rays on 8/30/2019
        # Option to visualize all of the rays used for planning linear
        # reaches.
        show_all_rays = False
        # Option to find plans that assume the robot can navigate
        # everywhere regardless of obstacles.
        navigate_everywhere = False
        # Reduce the number or rays to save computation. This will
        # result in some subsampling, especially in the ring in the
        # corner directions.
        num_angles =  num_angles #num_angles/4.0 #80 # dividing by 4 results in 400 rays on 8/30/2019
        # If using angular subsampling, this method can be used to
        # interpolate between rays using morphological closing. Note
        # that this does not currently interpolate the base angles.
        close_ray_plan = False

        num_angles = int(np.ceil(num_angles))
        
        print('num_angles =', num_angles)
        obstacle_image = dilated_finger_obstacle_image[:]
        print('pix_per_m =', pix_per_m)
        print('max_arm_travel_m =', self.planar_model.max_arm_travel_m)
        start_distance_m = reach_width_m / 2.0
        # pixel directions when the base is at 0 degrees (forward motion
        # of the base is to the right of the image)
        yaw_offset_left_m = (self.planar_model.gripper_and_arm_width_m / 2.0) - self.planar_model.radius_of_yaw_cylinder_m
        yaw_offset_length_m = self.planar_model.yaw_to_fingertips_m - start_distance_m
        origin_offset_left_m = yaw_offset_left_m + self.planar_model.yaw_axis_to_origin_left_m
        origin_offset_length_m = yaw_offset_length_m + self.planar_model.yaw_axis_to_origin_length_m
        if show_all_rays: 
            obstacle_image = np.zeros_like(obstacle_image)
            distance_map = np.ones_like(distance_map)
        if navigate_everywhere:
            distance_map = np.ones_like(distance_map)
        # Find base positions and angles from which the robot could
        # reach the target.
        base_xy_image, base_ang_image, arm_reach_image = numba_find_base_poses_that_reach_target(target_x, target_y, num_angles, pix_per_m,
                                                                                                 start_distance_m, self.planar_model.max_arm_travel_m,
                                                                                                 origin_offset_left_m, origin_offset_length_m,
                                                                                                 obstacle_image)
        if close_ray_plan: 
            # Morphologically close the rays to account for angular subsampling.
            kernel = np.ones((3,3), np.uint8)
            base_xy_image = cv2.morphologyEx(base_xy_image, cv2.MORPH_CLOSE, kernel)

        s = image.shape
        color_finger_obstacle_image = np.zeros([s[0], s[1], 3], np.uint8)
        color_finger_obstacle_image[:,:,0] = finger_obstacle_image
        color_finger_obstacle_image[base_xy_image > 0] = [0, 255, 0] 
        color_finger_obstacle_image[(base_xy_image == 255) & (distance_map > 0.0)] = [255, 255, 0]
        cv2.circle(color_finger_obstacle_image, (target_x, target_y), 3, [0,0,255], -1)

        # Find base position candidates to which the robot can
        # navigate and from which it can reach the target.
        navigate_and_reach_base_xy_selector = (base_xy_image == 255) & (distance_map > 0.0)
        navigate_and_reach_base_xy_image = np.uint8(navigate_and_reach_base_xy_selector)
        
        # Find base positions from which the gripper can be deployed
        # (yaw joint).
        deploy_radius_pix = int(np.ceil(self.planar_model.yaw_to_fingertips_m * pix_per_m))
        kernel_width_pix = 1 + (2 * deploy_radius_pix)
        kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
        cv2.circle(kernel, (deploy_radius_pix, deploy_radius_pix), deploy_radius_pix, 255, -1)
        dilated_finger_obstacle_image = cv2.dilate(finger_obstacle_image, kernel)
        # These planar model parameters are always positive. The
        # direction is handled in the numba code.
        origin_to_yaw_left_m = self.planar_model.yaw_axis_to_origin_left_m 
        origin_to_yaw_length_m = self.planar_model.yaw_axis_to_origin_length_m
        tool_deploy_base_xy_image = numba_check_that_tool_can_deploy(navigate_and_reach_base_xy_image, base_ang_image,
                                                                     dilated_finger_obstacle_image,
                                                                     origin_to_yaw_left_m, origin_to_yaw_length_m,
                                                                     pix_per_m)

        
        # Find the the candidate with the most distance between it and
        # any of the boundaries of the connected base pose candidates
        # that can also deploy the gripper.

        # Distances of candidates to boundaries of connected base
        # positions.
        distance_map = cv2.distanceTransform(navigate_and_reach_base_xy_image, cv2.DIST_L2, 5)

        # Disallow base positions from which the gripper cannot deploy.
        distance_map[tool_deploy_base_xy_image == 0] = 0.0

        # Find the best remaining base position.
        min_val, max_val, (min_x, min_y), (max_x, max_y) = cv2.minMaxLoc(distance_map)
        base_x = max_x
        base_y = max_y
        base_ang = base_ang_image[base_y, base_x]
        arm_reach_pix = arm_reach_image[base_y, base_x]
        arm_reach_m = m_per_pix * arm_reach_pix
        base_ang_deg = 180.0 * (base_ang / np.pi)
        if max_val <= 0.0:
            print('No valid base pose candidate found.')
            found = False
        else: 
            print('Best best pose found: ({0}, {1}, {2:.2f}) = (x, y, theta_deg)'.format(base_x, base_y, base_ang_deg))
            print('     reach length = {0:.2f} m'.format(arm_reach_m))
            found = True
        
        if image_display_on:
            norm_dist_transform = cv2.normalize(distance_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)        
            cv2.imshow('distance map without threshold for the robot width', norm_dist_transform)

            s = image.shape
            color_height_image = np.zeros([s[0], s[1], 3], np.uint8)
            color_height_image[:,:,0] = image
            color_height_image[navigate_and_reach_base_xy_selector] = [255, 255, 0]
            color_height_image[tool_deploy_base_xy_image > 0] = [255, 255, 255]
            cv2.circle(color_height_image, (target_x, target_y), 3, [0,0,255], -1)
            cv2.circle(color_height_image, (base_x, base_y), 3, [0,255,0], -1)

            max_height_image.print_info()
            if image_display_on:
                cv2.imshow('finger obstacle image', finger_obstacle_image)
                cv2.imshow('color height image', color_height_image)
                base_ang_image_uint8 = np.uint8(255.0 * ((base_ang_image - np.min(base_ang_image)) / (np.max(base_ang_image) - np.min(base_ang_image))))

            print('Finished. Now displaying and waiting for user input to terminate.')

        if not found:
            return None, None, None, None
        else:
            return base_x, base_y, base_ang, arm_reach_m
