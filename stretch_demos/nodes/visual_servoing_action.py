#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Transform, TransformStamped, Pose
import ros_numpy
import numpy as np
import tf2_ros
import time
import actionlib
from math import sqrt, pow, acos, degrees
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from stretch_demos.msg import VisualServoAction, VisualServoGoal, VisualServoResult, VisualServoFeedback

from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
import hello_helpers.hello_misc as hm

class VisualServoing(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'dock_visual_servo', 'dock_visual_servo', wait_for_first_pointcloud=False)

        self.cmd_vel_pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=10)

        self.server = actionlib.SimpleActionServer('autodock_visual_servo', VisualServoAction, self.execute_cb, False)
        self.goal = VisualServoGoal()
        self.feedback = VisualServoFeedback()
        self.result = VisualServoResult()
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0
        self.server.start()

        self.distance = 0.0
        self.angle = 0.0
        self.theta = 0.0 # angle between baselink x-axis and predock_pose x-axis
        self.phi = 0.0 # angle between line connecting docking station and predock_pose and line connecting docking station with base_link

    def execute_cb(self, goal):
        self.goal = goal

        scan_point = JointTrajectoryPoint()
        scan_point.time_from_start = rospy.Duration(2.0)
        scan_point.positions = [-1.051, -2.914] # Look back and down at the docking station

        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.trajectory.joint_names = ['joint_head_tilt', 'joint_head_pan']
        trajectory_goal.trajectory.points = [scan_point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        trajectory_goal.trajectory.header.frame_id = 'base_link'

        self.trajectory_client.send_goal(trajectory_goal)
        self.trajectory_client.wait_for_result()
        
        while self.distance > 0.1 and self.distance < 0.5:
            try:
                predock_trans = self.tf2_buffer.lookup_transform('docking_station', 'predock_pose', rospy.Time(), rospy.Duration(1.0))
                base_trans = self.tf2_buffer.lookup_transform('docking_station', 'base_link', rospy.Time(), rospy.Duration(1.0))
                base_predock_trans = self.tf2_buffer.lookup_transform('predock_pose', 'base_link', rospy.Time(), rospy.Duration(1.0))

                x = predock_trans.transform.translation.x
                y = predock_trans.transform.translation.y
                z = predock_trans.transform.translation.z
                side_a = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))

                x = base_trans.transform.translation.x
                y = base_trans.transform.translation.y
                z = base_trans.transform.translation.z
                side_b = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))

                x = base_predock_trans.transform.translation.x
                y = base_predock_trans.transform.translation.y
                z = base_predock_trans.transform.translation.z
                side_c = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))

                if y < 0:
                    self.phi = -acos((pow(side_a, 2) + pow(side_b, 2) - pow(side_c, 2))/(2.0 * side_a * side_b))
                else:
                    self.phi = acos((pow(side_a, 2) + pow(side_b, 2) - pow(side_c, 2))/(2.0 * side_a * side_b))

                angles = euler_from_quaternion([base_predock_trans.transform.rotation.x,
                                                    base_predock_trans.transform.rotation.y,
                                                    base_predock_trans.transform.rotation.z,
                                                    base_predock_trans.transform.rotation.w])
                self.theta = -angles[2]
                
                self.angle = self.phi + self.theta
                rospy.loginfo("Angle obtained: {}".format(self.angle))

                # if self.angle > 0.05 or self.angle < -0.05:
                if self.angle > (self.theta + 0.05) or self.angle < (self.theta - 0.05):
                    self.cmd_vel.angular.z = self.angle*0.5
                else:
                    self.cmd_vel.angular.z = 0.0
                self.cmd_vel.linear.x = -(self.distance*0.25)
                self.cmd_vel_pub.publish(self.cmd_vel)
                self.feedback.angle_error = self.angle
                self.feedback.distance_error = self.distance
                self.server.publish_feedback(self.feedback)
            except:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)

        if self.distance > 0.5:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            self.result.result = False
            self.result_cb(self.result.result)
        else:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            self.result.result = True
            self.result_cb(self.result.result)
        

    def result_cb(self, result):
        if result:    
            rospy.loginfo("Docking complete")
            self.server.set_succeeded(self.result)
        else:
            rospy.loginfo("Failed to dock")
            self.server.set_aborted(self.result)

    def aruco_callback(self, msg):
        self.markers = msg.markers

    def main(self):
        self.rate = 10.0
        rate = rospy.Rate(self.rate)
        
        self.tf2_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        charging_port = TransformStamped()
        charging_port.transform.translation.x = -0.3
        charging_port.transform.translation.y = 0.0
        charging_port.transform.translation.z = 0.1
        charging_port.transform.rotation.x = 0.0
        charging_port.transform.rotation.y = 0.0
        charging_port.transform.rotation.z = 1.0
        charging_port.transform.rotation.w = 0.0
        charging_port.header.frame_id = 'base_link'
        charging_port.child_frame_id = 'charging_port'
        time.sleep(1.0)
        while not rospy.is_shutdown():
            try:
                charging_port.header.stamp = rospy.Time.now()
                self.tf2_broadcaster.sendTransform(charging_port)
                transform = self.tf2_buffer.lookup_transform('charging_port', 'docking_station', rospy.Time(), rospy.Duration(1.0))
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                z = transform.transform.translation.z
                self.distance = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))
                # rospy.loginfo("distance to docking_station {}".format(self.distance))
            except (AttributeError, tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                pass
            rate.sleep()


def main():
    try:
        node = VisualServoing()
        node.main()
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
