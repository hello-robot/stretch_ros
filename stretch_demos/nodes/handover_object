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

from visualization_msgs.msg import MarkerArray, Marker

from geometry_msgs.msg import PointStamped

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import math
import time
import threading
import sys

import tf2_ros
import argparse as ap        
import numpy as np
import threading
import ros_numpy as rn

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv


class HandoverObjectNode(hm.HelloNode):

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
        
        marker = Marker()
        self.mouth_marker_type = marker.CUBE
        self.mouth_point = None

        num_pan_angles = 5

        # looking out along the arm
        middle_pan_angle = -math.pi/2.0

        look_around_range = math.pi/3.0
        min_pan_angle = middle_pan_angle - (look_around_range / 2.0)
        max_pan_angle = middle_pan_angle + (look_around_range / 2.0)
        pan_angle = min_pan_angle
        pan_increment = look_around_range / float(num_pan_angles - 1.0)
        self.pan_angles = [min_pan_angle + (i * pan_increment)
                           for i in range(num_pan_angles)]
        self.pan_angles = self.pan_angles + self.pan_angles[1:-1][::-1]
        self.prev_pan_index = 0
        
        self.move_lock = threading.Lock()

        with self.move_lock: 
            self.handover_goal_ready = False
        
    def joint_states_callback(self, joint_states):
        with self.joint_states_lock: 
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position

    def look_around_callback(self):
        # Cycle the head back and forth looking for a person to whom
        # to handout the object.
        with self.move_lock:
            pan_index = (self.prev_pan_index + 1) % len(self.pan_angles)
            pan_angle = self.pan_angles[pan_index]
            pose = {'joint_head_pan': pan_angle}
            self.move_to_pose(pose)
            self.prev_pan_index = pan_index
    
    def mouth_position_callback(self, marker_array):
        with self.move_lock: 

            for marker in marker_array.markers:
                if marker.type == self.mouth_marker_type:
                    mouth_position = marker.pose.position
                    self.mouth_point = PointStamped()
                    self.mouth_point.point = mouth_position
                    header = self.mouth_point.header
                    header.stamp = marker.header.stamp
                    header.frame_id = marker.header.frame_id
                    header.seq = marker.header.seq
                    print('******* new mouth point received *******')

                    lookup_time = rospy.Time(0) # return most recent transform
                    timeout_ros = rospy.Duration(0.1)

                    old_frame_id = self.mouth_point.header.frame_id[1:]
                    new_frame_id = 'base_link'
                    stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, old_frame_id, lookup_time, timeout_ros)
                    points_in_old_frame_to_new_frame_mat = rn.numpify(stamped_transform.transform)
                    camera_to_base_mat = points_in_old_frame_to_new_frame_mat

                    grasp_center_frame_id = 'link_grasp_center'
                    stamped_transform = self.tf2_buffer.lookup_transform(new_frame_id, grasp_center_frame_id, lookup_time, timeout_ros)
                    grasp_center_to_base_mat = rn.numpify(stamped_transform.transform)

                    mouth_camera_xyz = np.array([0.0, 0.0, 0.0, 1.0])
                    mouth_camera_xyz[:3] = rn.numpify(self.mouth_point.point)[:3]

                    mouth_xyz = np.matmul(camera_to_base_mat, mouth_camera_xyz)[:3]
                    fingers_xyz = grasp_center_to_base_mat[:,3][:3]

                    handoff_object = True

                    if handoff_object:
                        # attempt to handoff the object at a location below
                        # the mouth with respect to the world frame (i.e.,
                        # gravity)
                        target_offset_xyz = np.array([0.0, 0.0, -0.2])
                    else: 
                        object_height_m = 0.1
                        target_offset_xyz = np.array([0.0, 0.0, -object_height_m])
                    target_xyz = mouth_xyz + target_offset_xyz

                    fingers_error = target_xyz - fingers_xyz
                    print('fingers_error =', fingers_error)

                    delta_forward_m = fingers_error[0] 
                    delta_extension_m = -fingers_error[1]
                    delta_lift_m = fingers_error[2]

                    max_lift_m = 1.0
                    lift_goal_m = self.lift_position + delta_lift_m
                    lift_goal_m = min(max_lift_m, lift_goal_m)
                    self.lift_goal_m = lift_goal_m

                    self.mobile_base_forward_m = delta_forward_m

                    max_wrist_extension_m = 0.5
                    wrist_goal_m = self.wrist_position + delta_extension_m

                    if handoff_object:
                        # attempt to handoff the object by keeping distance
                        # between the object and the mouth distance
                        #wrist_goal_m = wrist_goal_m - 0.3 # 30cm from the mouth
                        wrist_goal_m = wrist_goal_m - 0.25 # 25cm from the mouth
                        wrist_goal_m = max(0.0, wrist_goal_m)

                    self.wrist_goal_m = min(max_wrist_extension_m, wrist_goal_m)

                    self.handover_goal_ready = True

            
    def trigger_handover_object_callback(self, request):
        with self.move_lock: 
            # First, retract the wrist in preparation for handing out an object.
            pose = {'wrist_extension': 0.005}
            self.move_to_pose(pose)

            if self.handover_goal_ready: 
                pose = {'joint_lift': self.lift_goal_m}
                self.move_to_pose(pose)
                tolerance_distance_m = 0.01
                at_goal = self.move_base.forward(self.mobile_base_forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
                pose = {'wrist_extension': self.wrist_goal_m}
                self.move_to_pose(pose)
                self.handover_goal_ready = False

            return TriggerResponse(
                success=True,
                message='Completed object handover!'
                )

    
    def main(self):
        hm.HelloNode.main(self, 'handover_object', 'handover_object', wait_for_first_pointcloud=False)

        self.joint_states_subscriber = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        
        self.trigger_deliver_object_service = rospy.Service('/deliver_object/trigger_deliver_object',
                                                            Trigger,
                                                            self.trigger_handover_object_callback)
        
        self.mouth_position_subscriber = rospy.Subscriber('/nearest_mouth/marker_array', MarkerArray, self.mouth_position_callback)
        
        # This rate determines how quickly the head pans back and forth.
        rate = rospy.Rate(0.5)
        look_around = False
        while not rospy.is_shutdown():
            if look_around: 
                self.look_around_callback()
            rate.sleep()

        
if __name__ == '__main__':
    try:
        parser = ap.ArgumentParser(description='Handover an object.')
        args, unknown = parser.parse_known_args()
        node = HandoverObjectNode()
        node.main()
    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
