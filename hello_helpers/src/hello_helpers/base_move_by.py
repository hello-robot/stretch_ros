#!/usr/bin/env python3

from stretch_body.robot import Robot
import hello_helpers.hello_misc as hm
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose2D, Twist
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import time
import sys
from math import pow, sqrt, pi

class BaseMoveBy(hm.HelloNode):
    def _init_(self):
        hm.HelloNode.__init__(self)
        self.joint_state = None
        self.robot_pose = None

    def base_translate(self, trans):
        print('Base translate command received')
        command = {'joint': 'translate_mobile_base', 'inc': trans}
        # self.send_command(command)
        self.base_controller(command)

    def base_rotate(self, rot):
        print('Base rotate command received')
        command = {'joint': 'rotate_mobile_base', 'inc': rot}
        # self.send_command(command)
        self.base_controller(command)

    def base_controller(self, command):
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        if command['joint'] == 'translate_mobile_base':
            x_i = self.robot_pose.x
            y_i = self.robot_pose.y
            dist_travelled = 0
            dist_err = command['inc']
            while abs(dist_err) >= 0.02:
                # limit translation velocity between 0.05 and 0.2 m/s
                t_vel = dist_err
                if command['inc'] > 0:
                    t_vel = min(t_vel, 0.2)
                    t_vel = max(t_vel, 0.05)
                else:
                    t_vel = min(t_vel, -0.05)
                    t_vel = max(t_vel, -0.2)

                self.twist.linear.x = t_vel
                self.base_vel.publish(self.twist)
                dist_travelled = sqrt(pow(x_i-self.robot_pose.x, 2) + pow(y_i-self.robot_pose.y, 2))
                if command['inc'] > 0:
                    dist_err = command['inc'] - dist_travelled
                else: # Could introduce a bug where if the goal is overshot, the robot keeps going ahead
                    dist_err = -(abs(command['inc']) - dist_travelled)

            self.twist.linear.x = 0.0
            self.base_vel.publish(self.twist)

        else:
            if command['inc'] > pi or command['inc'] < -pi:
                print("Requested angle out of bounds")
                print("The goal angle must be in the range -3.141 to 3.141")
                sys.exit(1)
            
            crossover = None
            theta_travelled = 0
            theta_i = self.robot_pose.theta
            if theta_i + command['inc'] < -pi:
                crossover = 'positive'
            if theta_i + command['inc'] > pi:
                crossover = 'negative'
            theta_err = command['inc']
            while abs(theta_err) >= 0.02:
                # limit rotational velocity between 0.05 rad/s and 0.25 rad/s
                r_vel = theta_err
                if theta_err > 0:
                    r_vel = min(r_vel, 0.25)
                    r_vel = max(r_vel, 0.05)
                else:
                    r_vel = min(r_vel, -0.05)
                    r_vel = max(r_vel, -0.25)

                self.twist.angular.z = r_vel
                self.base_vel.publish(self.twist)

                if crossover == 'negative':
                    if self.robot_pose.theta > 0:
                        theta_travelled = self.robot_pose.theta - theta_i
                    else:
                        theta_travelled = pi - theta_i + pi - abs(self.robot_pose.theta)
                elif crossover == 'positive':
                    if self.robot_pose.theta < 0:
                        theta_travelled = -self.robot_pose.theta + theta_i
                    else:
                        theta_travelled = pi + theta_i + pi - self.robot_pose.theta
                else:
                    if command['inc'] < 0:
                        theta_travelled = theta_i - self.robot_pose.theta
                    else:
                        theta_travelled = self.robot_pose.theta - theta_i

                if command['inc'] > 0:
                    theta_err = command['inc'] - theta_travelled
                else: # Could cause a bug where if the goal is overshot, the robot keeps turning
                    theta_err = -(abs(command['inc']) - theta_travelled)

            self.twist.angular.z = 0.0
            self.base_vel.publish(self.twist)

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def pose2d_callback(self, pose):
        self.robot_pose = pose

    def send_command(self, command):
        print('Moving the base')
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
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.trajectory_client.send_goal(trajectory_goal)

    def main(self):
        hm.HelloNode.main(self, 'base_control', 'base_control', wait_for_first_pointcloud=False)
        rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/pose2D', Pose2D, self.pose2d_callback)
        self.base_vel = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1) #/stretch_diff_drive_controller/cmd_vel for gazebo
        
        while not rospy.is_shutdown():
            time.sleep(2)
            self.base_translate(0.5)
            time.sleep(2)
            self.base_rotate(-1.57)
            self.base_rotate(-1.57)
            time.sleep(2)
            self.base_translate(0.5)
            time.sleep(2)
            self.base_rotate(-1.57)
            self.base_rotate(-1.57)


if __name__ == '__main__':
    base = BaseMoveBy()
    base.main()
