#! /usr/bin/env python

import hello_helpers.hello_misc as hm

import rospy
import logging
import tf
import tf2_ros
import time
from math import pi, sqrt, atan2
import json 
import os 

from std_srvs.srv import Trigger

import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

class ArucoNavigationNode(hm.HelloNode):
    def __init__(self):
        
        hm.HelloNode.__init__(self)

        self.joint_state = None
        
        self.file_path = rospy.get_param("/file_path")
        
        try:
            saved_file = open(self.file_path + "/saved_poses.json")
            self.pose_dict = json.load(saved_file)
            saved_file.close()
        except:
            self.pose_dict = {}
        

        self.main()

    def pose_to_list(self, pose):
        '''
        Reformats a PoseStamped message into a list that can be saved as a dictionary entry
        '''
        
        return [pose.header.frame_id, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]

    def list_to_pose(self, list):
        '''
        Reformats a pose dictionary entry into the original PoseStamped message
        '''
        
        pose = tf2_geometry_msgs.PoseStamped()
        pose.header.frame_id = list[0]
        pose.pose.position.x = list[1]
        pose.pose.position.y = list[2]
        pose.pose.position.z = list[3]
        pose.pose.orientation.x = list[4]
        pose.pose.orientation.y = list[5]
        pose.pose.orientation.z = list[6]
        pose.pose.orientation.w = list[7]
        
        return pose

    def joint_states_callback(self, joint_state):
        '''
        Callback for the /stretch/joint_states topic to store the current joint states for use within the class
        '''

        self.joint_state = joint_state

    def send_command(self, command):
        '''
        Handles single joint control commands by constructing a FollowJointTrajectoryGoal message and sending it to the trajectory_client 
        created in hello_misc.
        '''

        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.0)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
            
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            if 'inc' in command:
                inc = command['inc']
                new_value = inc
            elif 'delta' in command:
                joint_index = joint_state.name.index(joint_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.trajectory_client.send_goal(trajectory_goal)
            self.trajectory_client.wait_for_result()

    def find_tag(self, requested_tag):
        '''
        Pans the head at three tilt angles to search for the requested frame (usually either the name of an aruco tag or "map"). Cycles up for up to two full searches (a total of 6 rotations) before timing 
        out. If the frame is found, a tf_listener finds the pose of the base_link in the requested frame and saves it in the translation and rotation variables for use in the next functions.
        '''

        # self.switch_to_position_mode()

        min_rotation = -4.05
        max_rotation = 1.78
        num_steps = 10
        step = abs(min_rotation - max_rotation) / num_steps

        pose = {'joint_head_tilt': -1*pi/4, 'joint_head_pan': min_rotation}
        self.move_to_pose(pose)

        found_tag = False

        count = 0
        while not found_tag:
            
            command = {'joint': 'joint_head_pan', 'delta': step}
            self.send_command(command)
            time.sleep(.5)

            try:
                rospy.loginfo('LOOKING FOR THIS TAG: ')
                rospy.loginfo(requested_tag)
                self.translation, self.rotation = self.tf_listener.lookupTransform(requested_tag, 'base_link', rospy.Time(0))
                rospy.loginfo("Found Requested Tag")
                
                found_tag = True
            
            except:
                
                # Check if the head has completed a full rotation
                if self.joint_state.position[self.joint_state.name.index('joint_head_pan')] > (max_rotation - step):
                    
                    pose = {'joint_head_pan': min_rotation}
                    self.move_to_pose(pose)

                    # After a full head rotation, change the head tilt 
                    if self.joint_state.position[self.joint_state.name.index('joint_head_tilt')] >= -0.1:
                        pose = {'joint_head_tilt': -1*pi/4}
                        self.move_to_pose(pose)
                        count += 1
                    else:
                        command = {'joint': 'joint_head_tilt', 'delta': pi/8}
                        self.send_command(command)

                    time.sleep(.5)
    
            if count >= 2:
                rospy.loginfo("Timed Out Looking for Tag")
                # self.switch_to_navigation_mode()
                return False

        # self.switch_to_navigation_mode()
        return True

    
    def save_pose(self, pose_id, frame_id):
        '''
        Looks for the requested frame (the name of an aruco tag or "map") then returns the translation and rotation found by the tf_listener in find_tag as a pose
        in the requested frame.
        '''

        if self.find_tag(frame_id):
        
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            
            pose.pose.position.x = self.translation[0]
            pose.pose.position.y = self.translation[1]
            pose.pose.position.z = self.translation[2]

            pose.pose.orientation.x = self.rotation[0]
            pose.pose.orientation.y = self.rotation[1]
            pose.pose.orientation.z = self.rotation[2]
            pose.pose.orientation.w = self.rotation[3]

            saved_file = open(self.file_path + "/saved_poses.json","w")
            self.pose_dict[pose_id.lower()] = self.pose_to_list(pose)
            json.dump(self.pose_dict,saved_file)
            saved_file.close()

            return True

        else: 
            rospy.loginfo("Could not save pose")
            return False

    def go_to_pose(self, pose_id):
        '''
        Finds the requested pose in the saved pose dictionary, and sends a move_base goal to return to the given pose.
        '''
        
        if self.pose_dict.has_key(pose_id):

            pose = {'wrist_extension': 0.01}
            self.move_to_pose(pose)

            pose = {'joint_wrist_yaw': 3.3}
            self.move_to_pose(pose)
            
            pose = {'joint_lift': 0.22}
            self.move_to_pose(pose)
            
            pose_goal = self.list_to_pose(self.pose_dict[pose_id])
            tag = pose_goal.header.frame_id 
            if not self.find_tag(tag):
                print("Could not find tag")
                return False

            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)

            map_goal = MoveBaseGoal()
            while True:
                try:
                    map_goal.target_pose = tf_buffer.transform(pose_goal, 'map', rospy.Duration(0))
                    break
                except: 
                    if not self.find_tag(tag):
                        print("Could not find tag")
                        return False 
            
            map_goal.target_pose.pose.position.z = 0.0
            eul = tf.transformations.euler_from_quaternion((map_goal.target_pose.pose.orientation.x, map_goal.target_pose.pose.orientation.y, map_goal.target_pose.pose.orientation.z, map_goal.target_pose.pose.orientation.w))
            quat = tf.transformations.quaternion_from_euler(0.0, 0.0, eul[2])
            map_goal.target_pose.pose.orientation.x = quat[0]
            map_goal.target_pose.pose.orientation.y = quat[1]
            map_goal.target_pose.pose.orientation.z = quat[2]
            map_goal.target_pose.pose.orientation.w = quat[3]
            rospy.loginfo(map_goal)
            self.client.send_goal_and_wait(map_goal)
            rospy.loginfo("DONE!")

            return True
        else:
            print("Pose not found")
            return False

    def delete_pose(self, pose_id):
        '''
        Removes a requested pose from the saved pose dictionary
        '''

        if self.pose_dict.has_key(pose_id): 
            saved_file = open(self.file_path + "/saved_poses.json","w")
            
            del self.pose_dict[pose_id]
            
            json.dump(self.pose_dict,saved_file)
            saved_file.close()

            print("DELETED ", pose_id)

            return True 
        else: 
            print("Pose not found")
            return False

    def main(self):
        hm.HelloNode.main(self, 'save_pose', 'save_pose', wait_for_first_pointcloud=False)

        self.r = rospy.Rate(rospy.get_param('~rate', 15.0))

        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.tf_listener = tf.TransformListener()
        self.switch_to_position_mode = rospy.ServiceProxy('/switch_to_position_mode', Trigger)
        self.switch_to_navigation_mode = rospy.ServiceProxy('/switch_to_navigation_mode', Trigger)

if __name__ == '__main__':
    
    node = ArucoNavigationNode()
    node.main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
