#!/usr/bin/env python

from __future__ import print_function

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import math
import time
import threading
import sys
import tf2_ros
import argparse as ap        
import numpy as np
import os

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp


class GraspObjectNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.rate = 10.0
        self.joint_states = None
        self.joint_states_lock = threading.Lock()
        self.move_base = nv.MoveBase(self)
        self.letter_height_m = 0.2
        self.wrist_position = None
        self.lift_position = None
        self.manipulation_view = None
        self.debug_directory = None
        
    def joint_states_callback(self, joint_states):
        with self.joint_states_lock: 
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position
        self.left_finger_position, temp1, temp2 = hm.get_left_finger_state(joint_states)
        
    def lower_tool_until_contact(self):
        rospy.loginfo('lower_tool_until_contact')
        trigger_request = TriggerRequest() 
        trigger_result = self.trigger_lower_until_contact_service(trigger_request)
        rospy.loginfo('trigger_result = {0}'.format(trigger_result))
        
    def move_to_initial_configuration(self):
        initial_pose = {'wrist_extension': 0.01,
                        'joint_wrist_yaw': 0.0,
                        'gripper_aperture': 0.125}

        rospy.loginfo('Move to the initial configuration for drawer opening.')
        self.move_to_pose(initial_pose)

    def look_at_surface(self, scan_time_s=None):
        self.manipulation_view = mp.ManipulationView(self.tf2_buffer, self.debug_directory)
        manip = self.manipulation_view
        head_settle_time_s = 0.02 #1.0
        manip.move_head(self.move_to_pose)
        rospy.sleep(head_settle_time_s)
        if scan_time_s is None:
            manip.update(self.point_cloud, self.tf2_buffer)
        else:
            start_time_s = time.time()
            while ((time.time() - start_time_s) < scan_time_s): 
                manip.update(self.point_cloud, self.tf2_buffer)
        if self.debug_directory is not None:
            dirname = self.debug_directory + 'grasp_object/'
            # If the directory does not already exist, create it.
            if not os.path.exists(dirname):
                os.makedirs(dirname)
            filename = 'look_at_surface_' + hm.create_time_string()
            manip.save_scan(dirname + filename)
        else:
            rospy.loginfo('GraspObjectNode: No debug directory provided, so debugging data will not be saved.')

    def drive(self, forward_m):
        tolerance_distance_m = 0.005
        if forward_m > 0: 
            at_goal = self.move_base.forward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        else:
            at_goal = self.move_base.backward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        
    def trigger_grasp_object_callback(self, request):
        actually_move = True
        max_lift_m = 1.09
        min_extension_m = 0.01
        max_extension_m = 0.5
        
        use_default_mode = False
        if use_default_mode: 
            # Set the D435i to Default mode for obstacle detection
            trigger_request = TriggerRequest() 
            trigger_result = self.trigger_d435i_default_mode_service(trigger_request)
            rospy.loginfo('trigger_result = {0}'.format(trigger_result))

        if actually_move:
            rospy.loginfo('Retract the tool.')
            pose = {'wrist_extension': 0.01}
            self.move_to_pose(pose)

            rospy.loginfo('Reorient the wrist.')
            pose = {'joint_wrist_yaw': 0.0}
            self.move_to_pose(pose)
            
        self.look_at_surface(scan_time_s = 3.0)
        
        grasp_target = self.manipulation_view.get_grasp_target(self.tf2_buffer)
        
        if grasp_target is not None: 
            pregrasp_lift_m = self.manipulation_view.get_pregrasp_lift(grasp_target, self.tf2_buffer)

            if (self.lift_position is None):
                return TriggerResponse(
                    success=False,
                    message='lift position unavailable'
                )

            if actually_move:
                rospy.loginfo('Raise tool to pregrasp height.')
                lift_to_pregrasp_m = max(self.lift_position + pregrasp_lift_m, 0.1)
                lift_to_pregrasp_m = min(lift_to_pregrasp_m, max_lift_m)
                pose = {'joint_lift': lift_to_pregrasp_m}
                self.move_to_pose(pose)

            pregrasp_yaw = self.manipulation_view.get_pregrasp_yaw(grasp_target, self.tf2_buffer)
            print('pregrasp_yaw = {0:.2f} rad'.format(pregrasp_yaw))
            print('pregrasp_yaw = {0:.2f} deg'.format(pregrasp_yaw * (180.0/np.pi)))

            if actually_move:
                rospy.loginfo('Rotate the gripper for grasping.')
                pose = {'joint_wrist_yaw': pregrasp_yaw}
                self.move_to_pose(pose)
                
                rospy.loginfo('Open the gripper.')
                pose = {'gripper_aperture': 0.125}
                self.move_to_pose(pose)

            pregrasp_mobile_base_m, pregrasp_wrist_extension_m = self.manipulation_view.get_pregrasp_planar_translation(grasp_target, self.tf2_buffer)
            
            print('pregrasp_mobile_base_m = {0:.3f} m'.format(pregrasp_mobile_base_m))
            print('pregrasp_wrist_extension_m = {0:.3f} m'.format(pregrasp_wrist_extension_m))

            if actually_move:
                rospy.loginfo('Drive to pregrasp location.')
                self.drive(pregrasp_mobile_base_m)

                if pregrasp_wrist_extension_m > 0.0:
                    extension_m = max(self.wrist_position + pregrasp_wrist_extension_m, min_extension_m)
                    extension_m = min(extension_m, max_extension_m)
                    rospy.loginfo('Extend tool above surface.')
                    pose = {'wrist_extension': extension_m} 
                    self.move_to_pose(pose)
                else:
                    print('negative wrist extension for pregrasp, so not extending or retracting.')

            grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m = self.manipulation_view.get_grasp_from_pregrasp(grasp_target, self.tf2_buffer)
            print('grasp_mobile_base_m = {0:3f} m, grasp_lift_m = {1:3f} m, grasp_wrist_extension_m = {2:3f} m'.format(grasp_mobile_base_m, grasp_lift_m, grasp_wrist_extension_m))

            if actually_move: 
                rospy.loginfo('Move the grasp pose from the pregrasp pose.')

                lift_m = max(self.lift_position + grasp_lift_m, 0.1)
                lift_m = min(lift_m, max_lift_m)
                
                extension_m = max(self.wrist_position + grasp_wrist_extension_m, min_extension_m)
                extension_m = min(extension_m, max_extension_m)
                
                pose = {'translate_mobile_base': grasp_mobile_base_m,
                        'joint_lift': lift_m,  
                        'wrist_extension': extension_m}
                self.move_to_pose(pose)

                rospy.loginfo('Attempt to close the gripper on the object.')
                gripper_aperture_m = grasp_target['width_m'] - 0.18
                pose = {'gripper_aperture': gripper_aperture_m}
                self.move_to_pose(pose)
                
                # Lifting appears to happen before the gripper has
                # finished unless there is this sleep. Need to look
                # into this issue.
                rospy.sleep(3.0)

                rospy.loginfo('Attempt to lift the object.')
                object_lift_height_m = 0.1

                lift_m = max(self.lift_position + object_lift_height_m, 0.2)
                lift_m = min(lift_m, max_lift_m)
                
                pose = {'joint_lift': lift_m}
                self.move_to_pose(pose)

                rospy.loginfo('Open the gripper a little to avoid overtorquing and overheating the gripper motor.')
                pose = {'gripper_aperture': gripper_aperture_m + 0.005}
                self.move_to_pose(pose)


            if actually_move:
                rospy.loginfo('Retract the tool.')
                pose = {'wrist_extension': 0.01}
                self.move_to_pose(pose)

                rospy.loginfo('Reorient the wrist.')
                pose = {'joint_wrist_yaw': 0.0}
                self.move_to_pose(pose)

        return TriggerResponse(
            success=True,
            message='Completed object grasp!'
            )

    
    def main(self):
        hm.HelloNode.main(self, 'grasp_object', 'grasp_object', wait_for_first_pointcloud=False)

        self.debug_directory = rospy.get_param('~debug_directory')
        rospy.loginfo('Using the following directory for debugging files: {0}'.format(self.debug_directory))


        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        
        self.trigger_grasp_object_service = rospy.Service('/grasp_object/trigger_grasp_object',
                                                           Trigger,
                                                           self.trigger_grasp_object_callback)

        rospy.wait_for_service('/funmap/trigger_reach_until_contact')
        rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_reach_until_contact.')
        self.trigger_reach_until_contact_service = rospy.ServiceProxy('/funmap/trigger_reach_until_contact', Trigger)

        rospy.wait_for_service('/funmap/trigger_lower_until_contact')
        rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_lower_until_contact.')
        self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact', Trigger)

        default_service = '/camera/switch_to_default_mode'
        high_accuracy_service = '/camera/switch_to_high_accuracy_mode'
        rospy.loginfo('Node ' + self.node_name + ' waiting to connect to ' + default_service + ' and ' + high_accuracy_service)
        rospy.wait_for_service(default_service)
        rospy.loginfo('Node ' + self.node_name + ' connected to ' + default_service)
        self.trigger_d435i_default_mode_service = rospy.ServiceProxy(default_service, Trigger)
        rospy.wait_for_service(high_accuracy_service)
        rospy.loginfo('Node ' + self.node_name + ' connected to'  + high_accuracy_service)
        self.trigger_d435i_high_accuracy_mode_service = rospy.ServiceProxy(high_accuracy_service, Trigger)
        
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            rate.sleep()

        
if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Grasp Object behavior for stretch.')
        args, unknown = parser.parse_known_args()
        node = GraspObjectNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')

