#!/usr/bin/env python3

import rospy
import actionlib

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, TransformStamped, PoseWithCovarianceStamped, PoseStamped, Pose, PointStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult, MoveBaseFeedback
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose

import numpy as np
import scipy.ndimage as nd
import cv2
import math
import time
import threading
import sys
import os
import copy

import tf_conversions
import ros_numpy
import tf2_ros

import argparse as ap

import hello_helpers.hello_misc as hm
import hello_helpers.hello_ros_viz as hr

import stretch_funmap.merge_maps as mm
import stretch_funmap.navigate as nv
import stretch_funmap.mapping as ma
import stretch_funmap.segment_max_height_image as sm
import stretch_funmap.navigation_planning as na
import stretch_funmap.manipulation_planning as mp


def create_map_to_odom_transform(t_mat):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'map'
    t.child_frame_id = 'odom'
    t.transform = ros_numpy.msgify(Transform, t_mat)
    return t


class ContactDetector():
    def __init__(self, get_joint_state, in_contact_func, move_increment=0.008):
        self.in_contact_func = in_contact_func

        # reach until contact related
        self.in_contact = False
        self.in_contact_position = None
        self.contact_state_lock = threading.Lock()
        self.contact_mode = None
        self.contact_mode_lock = threading.Lock()

        self.position = None
        self.av_effort = None
        self.av_effort_window_size = 3

        self.get_joint_state = get_joint_state
        self.direction_sign = None
        self.stopping_position = None

        self.min_av_effort_threshold = 10.0
        self.move_increment = move_increment

    def set_regulate_contact(self):
        with self.contact_mode_lock:
            return self.contact_mode == 'regulate_contact'

    def set_stopping_position(self, stopping_position, direction_sign):
        assert((direction_sign == -1) or (direction_sign == 1))
        self.stopping_position = stopping_position
        self.direction_sign = direction_sign

    def is_in_contact(self):
        with self.contact_state_lock:
            return self.in_contact

    def contact_position(self):
        with self.contact_state_lock:
            return self.in_contact_position

    def get_position(self):
        return self.position

    def passed_stopping_position(self):
        if (self.position is None) or (self.stopping_position is None):
            return False
        difference = self.stopping_position - self.position
        if int(np.sign(difference)) == self.direction_sign:
            return False
        return True

    def not_stopped(self):
        with self.contact_mode_lock:
            return self.contact_mode == 'stop_on_contact'

    def reset(self):
        with self.contact_state_lock:
            self.in_contact = False
            self.in_contact_position = None
        self.turn_off()
        self.stopping_position = None
        self.direction_sign = None

    def turn_off(self):
        with self.contact_mode_lock:
            self.contact_mode = None

    def turn_on(self):
        with self.contact_mode_lock:
            self.contact_mode = 'stop_on_contact'

    def update(self, joint_states, stop_the_robot_service):
        with self.contact_state_lock:
            self.in_contact = False
            self.in_contact_wrist_position = None

        position, velocity, effort = self.get_joint_state(joint_states)
        self.position = position

        # First, check that the stopping position, if defined, has not been passed
        if self.passed_stopping_position():
            trigger_request = TriggerRequest()
            trigger_result = stop_the_robot_service(trigger_request)
            with self.contact_mode_lock:
                self.contact_mode = 'passed_stopping_point'
            rospy.loginfo('stop_on_contact: stopping the robot due to passing the stopping position, position = {0}, stopping_position = {1}, direction_sign = {2}'.format(
                self.position, self.stopping_position, self.direction_sign))

        # Second, check that the effort thresholds have not been exceeded
        if self.av_effort is None:
            self.av_effort = effort
        else:
            self.av_effort = (((self.av_effort_window_size - 1.0) *
                              self.av_effort) + effort) / self.av_effort_window_size

        if self.in_contact_func(effort, self.av_effort):
            # Contact detected!
            with self.contact_state_lock:
                self.in_contact = True
                self.in_contact_position = self.position
            with self.contact_mode_lock:
                if self.contact_mode == 'stop_on_contact':
                    trigger_request = TriggerRequest()
                    trigger_result = stop_the_robot_service(trigger_request)
                    rospy.loginfo('stop_on_contact: stopping the robot due to detected contact, effort = {0}, av_effort = {1}'.format(
                        effort, self.av_effort))
                    self.contact_mode = 'regulate_contact'
                elif self.contact_mode == 'regulate_contact':
                    pass
        elif self.av_effort < self.min_av_effort_threshold:
            with self.contact_mode_lock:
                if self.contact_mode == 'regulate_contact':
                    pass
        else:
            pass

    def move_until_contact(self, joint_name, stopping_position, direction_sign, move_to_pose):
        self.reset()
        self.set_stopping_position(stopping_position, direction_sign)

        success = False
        message = 'Unknown result.'

        if not self.passed_stopping_position():
            # The target has not been passed
            self.turn_on()

            move_rate = rospy.Rate(5.0)
            move_increment = direction_sign * self.move_increment
            finished = False
            while self.not_stopped():
                position = self.get_position()
                if position is not None:
                    new_target = self.get_position() + move_increment
                    pose = {joint_name: new_target}
                    move_to_pose(pose, return_before_done=True)
                move_rate.sleep()

            if self.is_in_contact():
                # back off from the detected contact location

                contact_position = self.contact_position()
                if contact_position is not None:
                    new_target = contact_position - 0.001  # - 0.002
                else:
                    new_target = self.position() - 0.001  # - 0.002
                pose = {joint_name: new_target}
                move_to_pose(pose, return_before_done=False)
                rospy.loginfo(
                    'backing off after contact: moving away from surface to decrease force')
                success = True
                message = 'Successfully reached until contact.'
            else:
                success = False
                message = 'Terminated without detecting contact.'

        self.reset()

        return success, message


class FunmapNode(hm.HelloNode):

    def __init__(self, map_filename):
        hm.HelloNode.__init__(self)

        self.map_filename = map_filename
        self.debug_directory = None

        # This holds all the poses the robot's mobile base was in
        # while making scans merged into the map. They are defined
        # with respect to the map's image. One use of this list is to
        # fill in the robot's footprints as floor when producing a
        # floor mask for the purposes of navigations with the
        # assumption that the robot's base will only be on traversable
        # floor.
        self.robot_poses = []
        self.prev_nav_markers = None

        self.wrist_position = None

        self.use_hook = False  # True #False

        if self.use_hook:
            def extension_contact_func(effort, av_effort):
                single_effort_threshold = 38.0
                av_effort_threshold = 34.0

                if (effort >= single_effort_threshold):
                    rospy.loginfo('Extension single effort exceeded single_effort_threshold: {0} >= {1}'.format(
                        effort, single_effort_threshold))
                if (av_effort >= av_effort_threshold):
                    rospy.loginfo('Extension average effort exceeded av_effort_threshold: {0} >= {1}'.format(
                        av_effort, av_effort_threshold))

                return ((effort >= single_effort_threshold) or
                        (av_effort > av_effort_threshold))

            self.extension_contact_detector = ContactDetector(
                hm.get_wrist_state, extension_contact_func, move_increment=0.008)

        else:
            def extension_contact_func(effort, av_effort):
                single_effort_threshold = 40.0
                av_effort_threshold = 40.0

                if (effort >= single_effort_threshold):
                    rospy.loginfo('Extension single effort exceeded single_effort_threshold: {0} >= {1}'.format(
                        effort, single_effort_threshold))
                if (av_effort >= av_effort_threshold):
                    rospy.loginfo('Extension average effort exceeded av_effort_threshold: {0} >= {1}'.format(
                        av_effort, av_effort_threshold))

                return ((effort >= single_effort_threshold) or
                        (av_effort > av_effort_threshold))

            self.extension_contact_detector = ContactDetector(
                hm.get_wrist_state, extension_contact_func)

        def lift_contact_func(effort, av_effort):
            single_effort_threshold = 20.0
            av_effort_threshold = 20.0

            if (effort <= single_effort_threshold):
                rospy.loginfo('Lift single effort less than single_effort_threshold: {0} <= {1}'.format(
                    effort, single_effort_threshold))
            if (av_effort <= av_effort_threshold):
                rospy.loginfo('Lift average effort less than av_effort_threshold: {0} <= {1}'.format(
                    av_effort, av_effort_threshold))

            return ((effort <= single_effort_threshold) or
                    (av_effort < av_effort_threshold))

        self.lift_down_contact_detector = ContactDetector(
            hm.get_lift_state, lift_contact_func)

    def publish_map_point_cloud(self):
        if self.merged_map is not None:
            max_height_point_cloud = self.merged_map.max_height_im.to_point_cloud()
            self.point_cloud_pub.publish(max_height_point_cloud)

            pub_voi = True
            if pub_voi:
                marker = self.merged_map.max_height_im.voi.get_ros_marker(
                    duration=1000.0)
                self.voi_marker_pub.publish(marker)

    def publish_nav_plan_markers(self, line_segment_path, image_to_points_mat, clicked_frame_id):
        path_height_m = 0.2
        points = [np.matmul(image_to_points_mat, np.array(
            [p[0], p[1], path_height_m, 1.0]))[:3] for p in line_segment_path]
        points = [[p[0], p[1], path_height_m] for p in points]
        self.publish_path_markers(points, clicked_frame_id)

    def publish_path_markers(self, points, points_frame_id):
        path_height_m = 0.2
        points = [[p[0], p[1], path_height_m] for p in points]
        if self.prev_nav_markers is not None:
            # delete previous markers
            for m in self.prev_nav_markers.markers:
                m.action = m.DELETE
            self.navigation_plan_markers_pub.publish(self.prev_nav_markers)
        nav_markers = MarkerArray()
        duration_s = 1 * 60
        timestamp = rospy.Time.now()
        m = hr.create_line_strip(points, 0, points_frame_id, timestamp, rgba=[
                                 0.0, 1.0, 0.0, 1.0], line_width_m=0.05, duration_s=duration_s)
        nav_markers.markers.append(m)
        for i, p in enumerate(points):
            m = hr.create_sphere_marker(p, i+1, points_frame_id, timestamp, rgba=[
                                        1.0, 1.0, 1.0, 1.0], diameter_m=0.15, duration_s=duration_s)
            nav_markers.markers.append(m)
        self.navigation_plan_markers_pub.publish(nav_markers)
        self.prev_nav_markers = nav_markers

    def trigger_align_with_nearest_cliff_service_callback(self, request):
        manip = mp.ManipulationView(self.tf2_buffer, self.debug_directory)
        manip.move_head(self.move_to_pose)
        manip.update(self.point_cloud, self.tf2_buffer)
        if self.debug_directory is not None:
            dirname = self.debug_directory + 'align_with_nearest_cliff/'
            # If the directory does not already exist, create it.
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            filename = 'nearest_cliff_scan_' + hm.create_time_string()
            manip.save_scan(dirname + filename)
        else:
            rospy.loginfo(
                'FunmapNode trigger_align_with_nearest_cliff_service_callback: No debug directory provided, so debugging data will not be saved.')
        p0, p1, normal = manip.get_nearest_cliff('odom', self.tf2_buffer)
        if normal is not None:
            cliff_ang = np.arctan2(normal[1], normal[0])

            # Find the robot's current pose in the odom frame.
            xya, timestamp = self.get_robot_floor_pose_xya(floor_frame='odom')
            robot_ang = xya[2]
            align_arm_ang = robot_ang + (np.pi/2.0)

            # Find the angle that the robot should turn in order
            # to point toward the next waypoint.
            turn_ang = hm.angle_diff_rad(cliff_ang, align_arm_ang)

            # Command the robot to turn to point to the next
            # waypoint.
            at_goal = self.move_base.turn(
                turn_ang, publish_visualizations=True)
            if not at_goal:
                message_text = 'Failed to reach turn goal.'
                rospy.loginfo(message_text)
                success = False
                message = message_text
            else:
                success = True
                message = 'Aligned with the nearest edge.'
        else:
            success = False
            message = 'Failed to detect cliff.'

        return TriggerResponse(
            success=success,
            message=message
        )

    def joint_states_callback(self, joint_states):
        self.extension_contact_detector.update(
            joint_states, self.stop_the_robot_service)
        self.wrist_position = self.extension_contact_detector.get_position()

        self.lift_down_contact_detector.update(
            joint_states, self.stop_the_robot_service)
        self.lift_position = self.lift_down_contact_detector.get_position()

    def trigger_reach_until_contact_service_callback(self, request):
        manip = mp.ManipulationView(self.tf2_buffer, self.debug_directory)
        manip.move_head(self.move_to_pose)
        manip.update(self.point_cloud, self.tf2_buffer)
        if self.debug_directory is not None:
            dirname = self.debug_directory + 'reach_until_contact/'
            # If the directory does not already exist, create it.
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            filename = 'reach_until_contact_' + hm.create_time_string()
            manip.save_scan(dirname + filename)
        else:
            rospy.loginfo(
                'FunmapNode trigger_reach_until_contact_service_callback: No debug directory provided, so debugging data will not be saved.')

        if self.use_hook:
            tooltip_frame = 'link_hook'
        else:
            tooltip_frame = 'link_grasp_center'
        reach_m = manip.estimate_reach_to_contact_distance(
            tooltip_frame, self.tf2_buffer)
        rospy.loginfo('----------------')
        rospy.loginfo('reach_m = {0}'.format(reach_m))
        rospy.loginfo('----------------')

        # Be aggressive moving in observed freespace and cautious
        # moving toward a perceived obstacle or unknown region.
        success = False
        message = 'Unknown result.'
        if self.wrist_position is not None:
            # The current wrist position needs to be known in order
            # for a reach command to be sent.
            max_reach_target_m = 0.5
            if (reach_m is not None):
                reach_target_m = reach_m + self.wrist_position
            else:
                reach_target_m = None

            if (reach_target_m is None) or (reach_target_m > max_reach_target_m):
                # Either the observed reach target was too far for the
                # arm, in which case we assume that something strange
                # happened and reach cautiously over the full reach.

                # Or, a freespace reach was not observed, so reach
                # cautiously over the full reach.
                direction_sign = 1
                success, message = self.extension_contact_detector.move_until_contact(
                    'wrist_extension', max_reach_target_m, direction_sign, self.move_to_pose)
            else:
                # A freespace region was observed. Agressively move to
                # within a safe distance of the expected obstacle.

                safety_margin_m = 0.02
                safe_target_m = reach_target_m - safety_margin_m
                if self.use_hook:
                    safe_target_m = safe_target_m + 0.03
                if safe_target_m > self.wrist_position:
                    pose = {'wrist_extension': safe_target_m}
                    self.move_to_pose(pose, return_before_done=False)

                # target depth within the surface
                target_depth_m = 0.08
                in_contact_target_m = reach_target_m + target_depth_m

                direction_sign = 1
                success, message = self.extension_contact_detector.move_until_contact(
                    'wrist_extension', in_contact_target_m, direction_sign, self.move_to_pose)

        return TriggerResponse(
            success=success,
            message=message
        )

    def trigger_lower_until_contact_service_callback(self, request):
        direction_sign = -1
        lowest_allowed_m = 0.3
        success, message = self.lift_down_contact_detector.move_until_contact(
            'joint_lift', lowest_allowed_m, direction_sign, self.move_to_pose)
        return TriggerResponse(
            success=success,
            message=message
        )

    def trigger_global_localization_service_callback(self, request):
        self.perform_head_scan(localize_only=True, global_localization=True)
        return TriggerResponse(
            success=True,
            message='Completed localization with scan.'
        )

    def trigger_local_localization_service_callback(self, request):
        self.perform_head_scan(
            localize_only=True, global_localization=False, fast_scan=True)
        return TriggerResponse(
            success=True,
            message='Completed localization with scan.'
        )

    def trigger_head_scan_service_callback(self, request):
        self.perform_head_scan()
        return TriggerResponse(
            success=True,
            message='Completed head scan.'
        )

    def trigger_drive_to_scan_service_callback(self, request):

        if self.merged_map is None:
            return TriggerResponse(
                success=False,
                message='No map exists yet, so unable to drive to a good scan spot.'
            )

        max_height_im = self.merged_map.max_height_im

        robot_xy_pix, robot_ang_rad, timestamp = max_height_im.get_robot_pose_in_image(
            self.tf2_buffer)
        robot_xya_pix = [robot_xy_pix[0], robot_xy_pix[1], robot_ang_rad]
        robot_x_pix = int(round(robot_xy_pix[0]))
        robot_y_pix = int(round(robot_xy_pix[1]))

        # Define the target maximum observation distance for any
        # observed point in the map. This serves as a goal for mapping.
        max_scan_distance_m = 1.5

        # The best case minimum width of the robot in meters when moving forward and backward.
        min_robot_width_m = 0.34

        camera_height_m = 1.12

        floor_mask = sm.compute_floor_mask(max_height_im)

        # Select the next location on the map from which to
        # attempt to make a head scan.
        best_xy = na.select_next_scan_location(floor_mask, max_height_im, min_robot_width_m,
                                               robot_x_pix, robot_y_pix, robot_ang_rad,
                                               camera_height_m, max_scan_distance_m,
                                               display_on=False)
        if best_xy is None:
            return TriggerResponse(
                success=False,
                message='No good scan location was detected.'
            )

        # Plan an optimistic path on the floor to the next
        # location for scanning.
        end_xy = np.array(best_xy)

        success, message = self.navigate_to_map_pixel(
            end_xy, robot_xya_pix=robot_xya_pix, floor_mask=floor_mask)

        return TriggerResponse(
            success=success,
            message=message
        )

    def pose_to_map_pixel(self, pose_stamped):
        clicked_frame_id = pose_stamped.header.frame_id
        clicked_timestamp = pose_stamped.header.stamp
        clicked_point = pose_stamped.pose.position

        # Check if a map exists
        if self.merged_map is None:
            success = False
            message = 'No map exists yet, so unable to drive to a good scan spot.'
            rospy.logerr(message)
            return None

        max_height_im = self.merged_map.max_height_im
        map_frame_id = self.merged_map.max_height_im.voi.frame_id

        points_to_image_mat, pi_timestamp = max_height_im.get_points_to_image_mat(
            clicked_frame_id, self.tf2_buffer)
        # lookup_time=clicked_timestamp)

        if (points_to_image_mat is not None):
            c_x = clicked_point.x
            c_y = clicked_point.y
            c_z = clicked_point.z
            clicked_xyz = np.array([c_x, c_y, c_z, 1.0])
            clicked_image_pixel = np.matmul(points_to_image_mat, clicked_xyz)
            i_x, i_y, i_z = clicked_image_pixel[:3]
            rospy.loginfo('clicked_image_pixel =' + str(clicked_image_pixel))
            end_xy = np.int64(np.round(np.array([i_x, i_y])))
            rospy.loginfo('end_xy =' + str(end_xy))
            return end_xy

        return None

    def plan_a_path(self, end_xy_pix, robot_xya_pix=None, floor_mask=None):
        # Transform the robot's current estimated pose as represented
        # by TF2 to the map image. Currently, the estimated pose is
        # based on the transformation from the map frame to the
        # base_link frame, which is updated by odometry and
        # corrections based on matching head scans to the map.
        path = None

        # Check if a map exists
        if self.merged_map is None:
            message = 'No map exists yet, so unable to drive to a good scan spot.'
            return path, message

        max_height_im = self.merged_map.max_height_im
        if robot_xya_pix is None:
            robot_xy_pix, robot_ang_rad, timestamp = max_height_im.get_robot_pose_in_image(
                self.tf2_buffer)
            robot_xya_pix = [robot_xy_pix[0], robot_xy_pix[1], robot_ang_rad]

        max_height_im = self.merged_map.max_height_im
        line_segment_path, message = na.plan_a_path(max_height_im, robot_xya_pix,
                                                    end_xy_pix, floor_mask=floor_mask)
        return line_segment_path, message

    def plan_to_reach(self, reach_xyz_pix, robot_xya_pix=None, floor_mask=None):
        # This is intended to perform coarse positioning of the
        # gripper near a target 3D point.
        robot_reach_xya_pix = None
        wrist_extension_m = None

        i_x, i_y, i_z = reach_xyz_pix

        max_height_im = self.merged_map.max_height_im
        # Check if a map exists
        if self.merged_map is None:
            message = 'No map exists yet, so unable to plan a reach.'
            rospy.logerr(message)
            return None, None

        if robot_xya_pix is None:
            robot_xy_pix, robot_ang_rad, timestamp = max_height_im.get_robot_pose_in_image(
                self.tf2_buffer)
            robot_xya_pix = [robot_xy_pix[0], robot_xy_pix[1], robot_ang_rad]

        end_xy_pix = np.int64(np.round(np.array([i_x, i_y])))
        m_per_height_unit = max_height_im.m_per_height_unit
        # move the gripper to be above the target point
        extra_target_height_m = 0.01
        target_z = i_z + (extra_target_height_m / m_per_height_unit)
        target_z_m = target_z * m_per_height_unit
        target_xyz_pix = (end_xy_pix[0], end_xy_pix[1], target_z)

        image_display_on = False

        manipulation_planner = mp.ManipulationPlanner()
        base_x_pix, base_y_pix, base_ang_rad, wrist_extension_m = manipulation_planner.base_pose(max_height_im,
                                                                                                 target_xyz_pix,
                                                                                                 robot_xya_pix,
                                                                                                 image_display_on=image_display_on)
        if image_display_on:
            c = cv2.waitKey(0)

        if base_x_pix is None:
            rospy.logerr('No valid base pose found for reaching the target.')
            return None, None

        robot_reach_xya_pix = [base_x_pix, base_y_pix, base_ang_rad]

        base_link_point = max_height_im.get_pix_in_frame(
            np.array(reach_xyz_pix), 'base_link', self.tf2_buffer)

        simple_reach_plan = []

        # close the gripper
        simple_reach_plan.append({'joint_gripper_finger_left': 0.0})

        # move the lift to be at the height of the target
        # The fingers of the gripper touch the floor at a joint_lift
        # height of 0.0 m, so moving the lift link to the height of
        # the target will result in the fingers being at the height of
        # the target.
        height_m = base_link_point[2]
        safety_z_m = 0.0
        simple_reach_plan.append({'joint_lift': height_m + safety_z_m})

        # rotate the gripper to be in the center
        # of the swept volume of the wrist (a
        # little right of center when looking out
        # from the robot to the gripper)
        #simple_reach_plan.append({'joint_gripper': -0.25})
        simple_reach_plan.append({'joint_wrist_yaw': -0.25})

        # reach out to the target
        # Reach to a point that is not fully at the target.
        safety_reach_m = 0.1  # 10cm away from the target
        simple_reach_plan.append(
            {'wrist_extension': wrist_extension_m - safety_reach_m})

        return robot_reach_xya_pix, simple_reach_plan

    def reach_to_click_callback(self, clicked_msg):
        rospy.loginfo('clicked_msg =' + str(clicked_msg))

        clicked_frame_id = clicked_msg.header.frame_id
        clicked_timestamp = clicked_msg.header.stamp
        clicked_point = clicked_msg.point

        max_height_im = self.merged_map.max_height_im
        # Check if a map exists
        if self.merged_map is None:
            message = 'No map exists yet, so unable to plan a reach.'
            rospy.logerr(message)
            return

        points_to_image_mat, pi_timestamp = max_height_im.get_points_to_image_mat(
            clicked_frame_id, self.tf2_buffer)

        if points_to_image_mat is None:
            rospy.logerr('points_to_image_mat not found')
            return

        c_x = clicked_point.x
        c_y = clicked_point.y
        c_z = clicked_point.z
        clicked_xyz = np.array([c_x, c_y, c_z, 1.0])
        clicked_image_pixel = np.matmul(points_to_image_mat, clicked_xyz)[:3]
        i_x, i_y, i_z = clicked_image_pixel
        rospy.loginfo('clicked_image_pixel =' + str(clicked_image_pixel))

        h, w = max_height_im.image.shape
        if not ((i_x >= 0) and (i_y >= 0) and (i_x < w) and (i_y < h)):
            rospy.logerr(
                'clicked point does not fall within the bounds of the max_height_image')
            return

        robot_xy_pix, robot_ang_rad, timestamp = max_height_im.get_robot_pose_in_image(
            self.tf2_buffer)
        robot_xya_pix = [robot_xy_pix[0], robot_xy_pix[1], robot_ang_rad]

        reach_xyz_pix = clicked_image_pixel
        robot_reach_xya_pix, simple_reach_plan = self.plan_to_reach(
            reach_xyz_pix, robot_xya_pix=robot_xya_pix)

        success, message = self.navigate_to_map_pixel(robot_reach_xya_pix[:2],
                                                      end_angle=robot_reach_xya_pix[2],
                                                      robot_xya_pix=robot_xya_pix)

        if success:
            for pose in simple_reach_plan:
                self.move_to_pose(pose)
        else:
            rospy.logerr(message)
            rospy.logerr('Aborting reach attempt due to failed navigation')

        return

    def navigate_to_map_pixel(self, end_xy, end_angle=None, robot_xya_pix=None, floor_mask=None):
        # Set the D435i to Default mode for obstacle detection
        trigger_request = TriggerRequest()
        trigger_result = self.trigger_d435i_default_mode_service(
            trigger_request)
        rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Move the head to a pose from which the D435i can detect
        # obstacles near the front of the mobile base while moving
        # forward.
        self.move_base.head_to_forward_motion_pose()

        line_segment_path, message = self.plan_a_path(
            end_xy, robot_xya_pix=robot_xya_pix, floor_mask=floor_mask)
        if line_segment_path is None:
            success = False
            return success, message

        # Existence of the merged map is checked by plan_a_path, but
        # to avoid future issues I'm introducing this redundancy.
        if self.merged_map is None:
            success = False
            return success, 'No map available for planning and navigation.'
        max_height_im = self.merged_map.max_height_im
        map_frame_id = self.merged_map.max_height_im.voi.frame_id

        # Query TF2 to obtain the current estimated transformation
        # from the map image to the map frame.
        image_to_points_mat, ip_timestamp = max_height_im.get_image_to_points_mat(
            map_frame_id, self.tf2_buffer)

        if image_to_points_mat is not None:

            # Publish a marker array to visualize the line segment path.
            self.publish_nav_plan_markers(
                line_segment_path, image_to_points_mat, map_frame_id)

            # Iterate through the vertices of the line segment path,
            # commanding the robot to drive to them in sequence using
            # in place rotations and forward motions.
            successful = True
            for p0, p1 in zip(line_segment_path, line_segment_path[1:]):
                # Query TF2 to obtain the current estimated transformation
                # from the image to the odometry frame.
                image_to_odom_mat, io_timestamp = max_height_im.get_image_to_points_mat(
                    'odom', self.tf2_buffer)

                # Query TF2 to obtain the current estimated transformation
                # from the robot's base_link frame to the odometry frame.
                robot_to_odom_mat, ro_timestamp = hm.get_p1_to_p2_matrix(
                    'base_link', 'odom', self.tf2_buffer)

                # Navigation planning is performed with respect to a
                # odom frame height of 0.0, so the heights of
                # transformed points are 0.0. The simple method of
                # handling the heights below assumes that the odom
                # frame is aligned with the floor, so that ignoring
                # the z coordinate is approximately equivalent to
                # projecting a point onto the floor.

                # Convert the current and next waypoints from map
                # image pixel coordinates to the odom
                # frame.
                p0 = np.array([p0[0], p0[1], 0.0, 1.0])
                p0 = np.matmul(image_to_odom_mat, p0)[:2]
                p1 = np.array([p1[0], p1[1], 0.0, 1.0])
                next_point_xyz = np.matmul(image_to_odom_mat, p1)
                p1 = next_point_xyz[:2]

                # Find the robot's current pose in the odom frame.
                xya, timestamp = self.get_robot_floor_pose_xya()
                r0 = xya[:2]
                r_ang = xya[2]

                # Check how far the robot's current location is from
                # its current waypoint. The current waypoint is where
                # the robot would ideally be located.
                waypoint_tolerance_m = 0.25
                waypoint_error = np.linalg.norm(p0 - r0)
                rospy.loginfo('waypoint_error =' + str(waypoint_error))
                if waypoint_error > waypoint_tolerance_m:
                    message_text = 'Failed due to waypoint_error being above the maximum allowed error.'
                    rospy.loginfo(message_text)
                    success = False
                    message = message_text
                    return success, message

                # Find the angle in the odometry frame that would
                # result in the robot pointing at the next waypoint.
                travel_vector = p1 - r0
                travel_dist = np.linalg.norm(travel_vector)
                travel_ang = np.arctan2(travel_vector[1], travel_vector[0])
                rospy.loginfo('travel_dist =' + str(travel_dist))
                rospy.loginfo('travel_ang =' + str(travel_ang * (180.0/np.pi)))

                # Find the angle that the robot should turn in order
                # to point toward the next waypoint.
                turn_ang = hm.angle_diff_rad(travel_ang, r_ang)

                # Command the robot to turn to point to the next
                # waypoint.
                rospy.loginfo('robot turn angle in degrees =' +
                              str(turn_ang * (180.0/np.pi)))
                at_goal = self.move_base.turn(
                    turn_ang, publish_visualizations=True)
                if not at_goal:
                    message_text = 'Failed to reach turn goal.'
                    rospy.loginfo(message_text)
                    success = False
                    message = message_text
                    return success, message

                # The head seems to drift sometimes over time, such
                # that the obstacle detection region is no longer
                # observed resulting in false positives. Hopefully,
                # this will correct the situation.
                self.move_base.head_to_forward_motion_pose()

                # FOR FUTURE DEVELOPMENT OF LOCAL NAVIGATION
                testing_future_code = False
                if testing_future_code:
                    check_result = self.move_base.check_line_path(
                        next_point_xyz, 'odom')
                    rospy.loginfo(
                        'Result of check line path = {0}'.format(check_result))
                    local_path, local_path_frame_id = self.move_base.local_plan(
                        next_point_xyz, 'odom')
                    if local_path is not None:
                        rospy.loginfo(
                            'Found local path! Publishing markers for it!')
                        self.publish_path_markers(
                            local_path, local_path_frame_id)
                    else:
                        rospy.loginfo('Did not find a local path...')

                # Command the robot to move forward to the next waypoing.
                at_goal = self.move_base.forward(
                    travel_dist, publish_visualizations=True)
                if not at_goal:
                    message_text = 'Failed to reach forward motion goal.'
                    rospy.loginfo(message_text)
                    success = False
                    message = message_text
                    return success, message

                rospy.loginfo('Turn and forward motion succeeded.')

            if end_angle is not None:
                # If a final target angle has been provided, rotate
                # the robot to match the target angle.
                rospy.loginfo(
                    'Attempting to achieve the final target orientation.')

                # Find the robot's current pose in the map frame. This
                # assumes that the target angle has been specified
                # with respect to the map frame.
                xya, timestamp = self.get_robot_floor_pose_xya(
                    floor_frame='map')
                r_ang = xya[2]

                # Find the angle that the robot should turn in order
                # to point toward the next waypoint.
                turn_ang = hm.angle_diff_rad(end_angle, r_ang)

                # Command the robot to turn to point to the next
                # waypoint.
                rospy.loginfo('robot turn angle in degrees =' +
                              str(turn_ang * (180.0/np.pi)))
                at_goal = self.move_base.turn(
                    turn_ang, publish_visualizations=True)
                if not at_goal:
                    message_text = 'Failed to reach turn goal.'
                    rospy.loginfo(message_text)
                    success = False
                    message = message_text
                    return success, message

        success = True
        message = 'Completed drive to new scan location.'
        return success, message

    def perform_head_scan(self, fill_in_blindspot_with_second_scan=True, localize_only=False, global_localization=False, fast_scan=False):
        node = self

        trigger_request = TriggerRequest()
        trigger_result = self.trigger_d435i_high_accuracy_mode_service(
            trigger_request)
        rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        # Reduce the occlusion due to the arm and grabber. This is
        # intended to be run when the standard grabber is not holding
        # an object.
        ma.stow_and_lower_arm(node)

        # Create and perform a new full scan of the environment using
        # the head.
        head_scan = ma.HeadScan(voi_side_m=16.0)
        head_scan.execute_full(node, fast_scan=fast_scan)

        scaled_scan = None
        scaled_merged_map = None

        # Save the new head scan to disk.
        if self.debug_directory is not None:
            dirname = self.debug_directory + 'head_scans/'
            # If the directory does not already exist, create it.
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            filename = 'head_scan_' + hm.create_time_string()
            head_scan.save(dirname + filename)
        else:
            rospy.loginfo(
                'FunmapNode perform_head_scan: No debug directory provided, so debugging data will not be saved.')

        head_scan.make_robot_footprint_unobserved()
        save_merged_map = False

        if self.merged_map is None:
            # The robot does not currently have a map, so initialize
            # the map with the new head scan.
            rospy.loginfo(
                'perform_head_scan: No map available, so setting the map to be the scan that was just taken.')
            self.merged_map = head_scan
            robot_pose = [head_scan.robot_xy_pix[0],
                          head_scan.robot_xy_pix[1], head_scan.robot_ang_rad]
            self.robot_poses.append(robot_pose)
            self.localized = True
            save_merged_map = True
        else:
            if localize_only and (not global_localization):
                # The scan was performed to localize the robot locally.
                rospy.loginfo(
                    'perform_head_scan: Performing local localization.')
                use_full_size_scans = False
                if use_full_size_scans:
                    affine_matrix, original_robot_map_pose, corrected_robot_map_pose = mm.estimate_scan_1_to_scan_2_transform(head_scan,
                                                                                                                              self.merged_map,
                                                                                                                              display_on=False,
                                                                                                                              show_unaligned=False,
                                                                                                                              full_localization=False,
                                                                                                                              init_target=None,
                                                                                                                              grid_search=False,
                                                                                                                              small_search=False)
                else:
                    original_robot_map_frame_pose, corrected_robot_map_frame_pose, original_robot_map_image_pose, corrected_robot_map_image_pose, scaled_scan, scaled_merged_map = ma.localize_with_reduced_images(
                        head_scan, self.merged_map, global_localization=False, divisor=2, small_search=True)

                    corrected_robot_map_pose = corrected_robot_map_frame_pose
                    original_robot_map_pose = original_robot_map_frame_pose
                    # Save the scaled scans to disk for debugging.
                    if self.debug_directory is not None:
                        dirname = self.debug_directory + 'scaled_localization_scans/'
                        # If the directory does not already exist, create it.
                        if not os.path.exists(dirname):
                            os.makedirs(dirname)
                        time_string = hm.create_time_string()
                        filename = 'localization_scaled_head_scan_' + time_string
                        scaled_scan.save(dirname + filename)
                        filename = 'localization_scaled_merged_map_' + time_string
                        scaled_merged_map.save(dirname + filename)
                    else:
                        rospy.loginfo(
                            'FunmapNode perform_head_scan: No debug directory provided, so debugging data will not be saved.')
                self.localized = True
            elif (not self.localized) or (localize_only and global_localization):
                # The robot has not been localized with respect to the
                # current map or the scan was performed solely to
                # globally localize the robot. This attempts to
                # localize the robot on the map by reducing the sizes
                # of the scan and the map in order to more efficiently
                # search for a match globally.

                # This does not merge the new scan into the current map.
                rospy.loginfo(
                    'perform_head_scan: Performing global localization.')
                save_merged_map = False

                original_robot_map_frame_pose, corrected_robot_map_frame_pose, original_robot_map_image_pose, corrected_robot_map_image_pose, scaled_scan, scaled_merged_map = ma.localize_with_reduced_images(
                    head_scan, self.merged_map, global_localization=True, divisor=6)  # 4)
                corrected_robot_map_pose = corrected_robot_map_frame_pose
                original_robot_map_pose = original_robot_map_frame_pose
                self.localized = True

                # Save the scaled scans to disk for debugging.
                if self.debug_directory is not None:
                    dirname = self.debug_directory + 'scaled_localization_scans/'
                    # If the directory does not already exist, create it.
                    if not os.path.exists(dirname):
                        os.makedirs(dirname)
                    time_string = hm.create_time_string()
                    filename = 'localization_scaled_head_scan_' + time_string
                    scaled_scan.save(dirname + filename)
                    filename = 'localization_scaled_merged_map_' + time_string
                    scaled_merged_map.save(dirname + filename)
                else:
                    rospy.loginfo(
                        'FunmapNode perform_head_scan: No debug directory provided, so debugging data will not be saved.')
            else:
                # The robot has been localized with respect to the
                # current map, so proceed to merge the new head scan
                # into the map. This assumes that the robot's
                # estimated pose is close to its actual pose in the
                # map. It constrains the matching optimization to a
                # limited range of positions and orientations.
                rospy.loginfo('perform_head_scan: Performing local map merge.')
                original_robot_map_pose, corrected_robot_map_pose = mm.merge_scan_1_into_scan_2(
                    head_scan, self.merged_map)
                save_merged_map = True

            # Store the corrected robot pose relative to the map frame.
            self.robot_poses.append(corrected_robot_map_pose)

            self.correct_robot_pose(
                original_robot_map_pose, corrected_robot_map_pose)
            pub_robot_markers = True
            if pub_robot_markers:
                self.publish_corrected_robot_pose_markers(
                    original_robot_map_pose, corrected_robot_map_pose)

        if save_merged_map:
            # If the merged map has been updated, save it to disk.
            if self.debug_directory is not None:
                head_scans_dirname = self.debug_directory + 'head_scans/'
                # If the directory does not already exist, create it.
                if not os.path.exists(head_scans_dirname):
                    os.makedirs(head_scans_dirname)
                merged_maps_dirname = self.debug_directory + 'merged_maps/'
                # If the directory does not already exist, create it.
                if not os.path.exists(merged_maps_dirname):
                    os.makedirs(merged_maps_dirname)
                time_string = hm.create_time_string()
                if scaled_scan is not None:
                    filename = 'localization_scaled_head_scan_' + time_string
                    scaled_scan.save(head_scans_dirname + filename)
                if scaled_merged_map is not None:
                    filename = 'localization_scaled_merged_map_' + time_string
                    scaled_merged_map.save(merged_maps_dirname + filename)
                filename = 'merged_map_' + hm.create_time_string()
                self.merged_map.save(merged_maps_dirname + filename)
            else:
                rospy.loginfo(
                    'FunmapNode perform_head_scan: No debug directory provided, so debugging data will not be saved.')

        if fill_in_blindspot_with_second_scan and (not localize_only):
            # Turn the robot to the left in attempt to fill in its
            # blindspot due to its mast.
            turn_ang = (70.0/180.0) * np.pi

            # Command the robot to turn to point to the next
            # waypoint.
            rospy.loginfo('robot turn angle in degrees =' +
                          str(turn_ang * (180.0/np.pi)))
            at_goal = self.move_base.turn(
                turn_ang, publish_visualizations=True)
            if not at_goal:
                message_text = 'Failed to reach turn goal.'
                rospy.loginfo(message_text)
            self.perform_head_scan(fill_in_blindspot_with_second_scan=False)

    def get_plan_service_callback(self, request):
        # request.start, request.goal, request.tolerance
        goal_pose = request.goal
        end_xy = self.pose_to_map_pixel(goal_pose)
        if end_xy is None:
            message = 'Failed to convert pose to map pixel.'
            rospy.logerr(message)
            return
        path, message = self.plan_a_path(end_xy)
        plan = Path()
        header = plan.header
        time_stamp = rospy.Time.now()
        header.stamp = time_stamp
        header.frame_id = 'map'
        if path is None:
            rospy.logerr(message)
            return plan

        # Existence of the merged map is checked by plan_a_path, but
        # to avoid future issues I'm introducing this redundancy.
        if self.merged_map is None:
            success = False
            return success, 'No map available for planning and navigation.'
        max_height_im = self.merged_map.max_height_im
        map_frame_id = self.merged_map.max_height_im.voi.frame_id

        # Query TF2 to obtain the current estimated transformation
        # from the map image to the map frame.
        image_to_points_mat, ip_timestamp = max_height_im.get_image_to_points_mat(
            map_frame_id, self.tf2_buffer)

        if image_to_points_mat is None:
            rospy.logerr('image_to_points_mat unavailable via TF2')
            return plan

        path_height_m = 0.0
        for xyz in path:
            image_point = np.array([xyz[0], xyz[1], 0.0, 1.0])
            map_point = np.matmul(image_to_points_mat, image_point)
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = time_stamp
            p.pose.position.x = map_point[0]
            p.pose.position.y = map_point[1]
            p.pose.position.z = path_height_m
            plan.poses.append(p)
        return plan

    def correct_robot_pose(self, original_robot_map_pose_xya, corrected_robot_map_pose_xya):
        # Compute and broadcast the corrected transformation from
        # the map frame to the odom frame.
        print('original_robot_map_pose_xya =', original_robot_map_pose_xya)
        print('corrected_robot_map_pose_xya =', corrected_robot_map_pose_xya)
        x_delta = corrected_robot_map_pose_xya[0] - \
            original_robot_map_pose_xya[0]
        y_delta = corrected_robot_map_pose_xya[1] - \
            original_robot_map_pose_xya[1]
        ang_rad_correction = hm.angle_diff_rad(
            corrected_robot_map_pose_xya[2], original_robot_map_pose_xya[2])
        c = np.cos(ang_rad_correction)
        s = np.sin(ang_rad_correction)
        rot_mat = np.array([[c, -s], [s, c]])
        x_old, y_old, a_old = original_robot_map_pose_xya
        xy_old = np.array([x_old, y_old])
        tx, ty = np.matmul(rot_mat, -xy_old) + \
            np.array([x_delta, y_delta]) + xy_old
        t = np.identity(4)
        t[0, 3] = tx
        t[1, 3] = ty
        t[:2, :2] = rot_mat
        self.map_to_odom_transform_mat = np.matmul(
            t, self.map_to_odom_transform_mat)
        self.tf2_broadcaster.sendTransform(
            create_map_to_odom_transform(self.map_to_odom_transform_mat))

    def publish_corrected_robot_pose_markers(self, original_robot_map_pose_xya, corrected_robot_map_pose_xya):
        # Publish markers to visualize the corrected and
        # uncorrected robot poses on the map.
        timestamp = rospy.Time.now()
        markers = MarkerArray()
        ang_rad = corrected_robot_map_pose_xya[2]
        x_axis = [np.cos(ang_rad), np.sin(ang_rad), 0.0]
        x, y, a = corrected_robot_map_pose_xya
        point = [x, y, 0.1]
        rgba = [0.0, 1.0, 0.0, 0.5]
        m_id = 0
        m = hr.create_sphere_marker(
            point, m_id, 'map', timestamp, rgba=rgba, diameter_m=0.1, duration_s=0.0)
        markers.markers.append(m)
        m_id += 1
        m = hr.create_axis_marker(
            point, x_axis, m_id, 'map', timestamp, rgba, length=0.2, arrow_scale=3.0)
        markers.markers.append(m)
        m_id += 1
        x, y, a = original_robot_map_pose_xya
        point = [x, y, 0.1]
        rgba = [1.0, 0.0, 0.0, 0.5]
        m = hr.create_sphere_marker(
            point, m_id, 'map', timestamp, rgba=rgba, diameter_m=0.1, duration_s=0.0)
        markers.markers.append(m)
        m_id += 1
        m = hr.create_axis_marker(
            point, x_axis, m_id, 'map', timestamp, rgba, length=0.2, arrow_scale=3.0)
        markers.markers.append(m)
        m_id += 1
        self.marker_array_pub.publish(markers)

    def set_robot_pose_callback(self, pose_with_cov_stamped):
        rospy.loginfo(
            'Set robot pose called. This will set the pose of the robot on the map.')
        rospy.loginfo(pose_with_cov_stamped)

        original_robot_map_pose_xya, timestamp = self.get_robot_floor_pose_xya(
            floor_frame='map')

        pwcs = pose_with_cov_stamped
        frame_id = pwcs.header.frame_id
        timestamp = pwcs.header.stamp
        pose = pwcs.pose.pose

        if frame_id != 'map':
            lookup_time = rospy.Time(0)  # return most recent transform
            timeout_ros = rospy.Duration(0.1)
            stamped_transform = tf2_buffer.lookup_transform(
                'map', frame_id, lookup_time, timeout_ros)
            map_pose = do_transform_pose(pose, stamped_transform)
        else:
            map_pose = pose

        p = map_pose.position
        q = map_pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        x = p.x
        y = p.y
        z = p.z
        roll, pitch, yaw = euler_from_quaternion(q_list)
        corrected_robot_map_pose_xya = [x, y, yaw]

        self.correct_robot_pose(
            original_robot_map_pose_xya, corrected_robot_map_pose_xya)
        self.publish_corrected_robot_pose_markers(
            original_robot_map_pose_xya, corrected_robot_map_pose_xya)

    def navigate_to_goal_topic_callback(self, goal_pose):
        rospy.loginfo(
            'Navigate to goal simple navigate to goal topic received a command!')
        rospy.loginfo(goal_pose)

        end_xy = self.pose_to_map_pixel(goal_pose)
        if end_xy is None:
            message = 'Failed to convert pose to map pixel.'
            rospy.logerr(message)
            return
        success, message = self.navigate_to_map_pixel(end_xy)

        if success:
            rospy.loginfo(message)
        else:
            rospy.logerr(message)
        return

    def navigate_to_goal_action_callback(self, goal):
        # geometry_msgs/PoseStamped target_pose
        goal_pose = goal.target_pose
        rospy.loginfo(
            'Navigate to goal simple action server received a command!')
        rospy.loginfo(goal_pose)

        end_xy = self.pose_to_map_pixel(goal_pose)
        if end_xy is None:
            message = 'Failed to convert pose to map pixel.'
            rospy.logerr(message)
            self.navigate_to_goal_action_server.set_aborted()
            return
        success, message = self.navigate_to_map_pixel(end_xy)

        if success:
            result = MoveBaseResult()
            self.navigate_to_goal_action_server.set_succeeded(result)
        else:
            rospy.logerr(message)
            self.navigate_to_goal_action_server.set_aborted()
        return

    def main(self):
        hm.HelloNode.main(self, 'funmap', 'funmap')

        self.debug_directory = rospy.get_param('~debug_directory')

        self.merged_map = None
        self.localized = False

        if self.map_filename is not None:
            self.merged_map = ma.HeadScan.from_file(self.map_filename)
            self.localized = False

        ###########################
        # Related to move_base API
        self.navigate_to_goal_action_server = actionlib.SimpleActionServer('/move_base',
                                                                           MoveBaseAction,
                                                                           execute_cb=self.navigate_to_goal_action_callback,
                                                                           auto_start=False)
        self.navigate_to_goal_action_server.start()

        self.navigation_goal_subscriber = rospy.Subscriber('/move_base_simple/goal',
                                                           PoseStamped,
                                                           self.navigate_to_goal_topic_callback)

        self.set_robot_pose_subscriber = rospy.Subscriber(
            '/initialpose', PoseWithCovarianceStamped, self.set_robot_pose_callback)

        self.get_plan_service = rospy.Service('/make_plan',
                                              GetPlan,
                                              self.get_plan_service_callback)
        ###########################

        self.trigger_head_scan_service = rospy.Service('/funmap/trigger_head_scan',
                                                       Trigger,
                                                       self.trigger_head_scan_service_callback)
        self.trigger_drive_to_scan_service = rospy.Service('/funmap/trigger_drive_to_scan',
                                                           Trigger,
                                                           self.trigger_drive_to_scan_service_callback)
        self.trigger_global_localization_service = rospy.Service('/funmap/trigger_global_localization',
                                                                 Trigger,
                                                                 self.trigger_global_localization_service_callback)
        self.trigger_local_localization_service = rospy.Service('/funmap/trigger_local_localization',
                                                                Trigger,
                                                                self.trigger_local_localization_service_callback)

        self.trigger_align_with_nearest_cliff_service = rospy.Service('/funmap/trigger_align_with_nearest_cliff',
                                                                      Trigger,
                                                                      self.trigger_align_with_nearest_cliff_service_callback)

        self.trigger_reach_until_contact_service = rospy.Service('/funmap/trigger_reach_until_contact',
                                                                 Trigger,
                                                                 self.trigger_reach_until_contact_service_callback)

        self.trigger_lower_until_contact_service = rospy.Service('/funmap/trigger_lower_until_contact',
                                                                 Trigger,
                                                                 self.trigger_lower_until_contact_service_callback)

        self.reach_to_click_subscriber = rospy.Subscriber(
            '/clicked_point', PointStamped, self.reach_to_click_callback)

        default_service = '/camera/switch_to_default_mode'
        high_accuracy_service = '/camera/switch_to_high_accuracy_mode'
        rospy.loginfo('Node ' + self.node_name + ' waiting to connect to ' +
                      default_service + ' and ' + high_accuracy_service)
        rospy.wait_for_service(default_service)
        rospy.loginfo('Node ' + self.node_name +
                      ' connected to ' + default_service)
        self.trigger_d435i_default_mode_service = rospy.ServiceProxy(
            default_service, Trigger)
        rospy.wait_for_service(high_accuracy_service)
        rospy.loginfo('Node ' + self.node_name +
                      ' connected to' + high_accuracy_service)
        self.trigger_d435i_high_accuracy_mode_service = rospy.ServiceProxy(
            high_accuracy_service, Trigger)

        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        self.point_cloud_pub = rospy.Publisher(
            '/funmap/point_cloud2', PointCloud2, queue_size=1)
        self.voi_marker_pub = rospy.Publisher(
            '/funmap/voi_marker', Marker, queue_size=1)
        self.marker_array_pub = rospy.Publisher(
            '/funmap/marker_array', MarkerArray, queue_size=1)
        self.navigation_plan_markers_pub = rospy.Publisher(
            '/funmap/navigation_plan_markers', MarkerArray, queue_size=1)
        self.obstacle_point_cloud_pub = rospy.Publisher(
            '/funmap/obstacle_point_cloud2', PointCloud2, queue_size=1)

        self.joint_states_subscriber = rospy.Subscriber(
            '/stretch/joint_states', JointState, self.joint_states_callback)

        self.rate = 5.0
        rate = rospy.Rate(self.rate)

        self.move_base = nv.MoveBase(self, self.debug_directory)

        self.map_to_odom_transform_mat = np.identity(4)
        while not rospy.is_shutdown():
            self.tf2_broadcaster.sendTransform(
                create_map_to_odom_transform(self.map_to_odom_transform_mat))
            self.publish_map_point_cloud()
            rate.sleep()


if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(
            description='Keyboard teleoperation for stretch.')
        parser.add_argument('--load_map', default=None,
                            help='Provide directory from which to load a map.')
        args, unknown = parser.parse_known_args()
        map_filename = args.load_map if args.load_map else None
        node = FunmapNode(map_filename)
        node.main()
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
