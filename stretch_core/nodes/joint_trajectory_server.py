#! /usr/bin/env python
from __future__ import print_function

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult

from command_groups import HeadPanCommandGroup, HeadTiltCommandGroup, \
                           WristYawCommandGroup, GripperCommandGroup, \
                           TelescopingCommandGroup, LiftCommandGroup, \
                           MobileBaseCommandGroup


class JointTrajectoryAction:

    def __init__(self, node):
        self.node = node
        self.server = actionlib.SimpleActionServer('/stretch_controller/follow_joint_trajectory',
                                                   FollowJointTrajectoryAction,
                                                   execute_cb=self.execute_cb,
                                                   auto_start=False)
        self.feedback = FollowJointTrajectoryFeedback()
        self.result = FollowJointTrajectoryResult()

        self.telescoping_cg = TelescopingCommandGroup(self.node.wrist_extension_calibrated_retracted_offset_m)
        if self.node.use_lift:
            self.lift_cg = LiftCommandGroup(self.node.max_arm_height)
        self.mobile_base_cg = MobileBaseCommandGroup()
        self.head_pan_cg = HeadPanCommandGroup(self.node.head_pan_calibrated_offset_rad,
                                               self.node.head_pan_calibrated_looked_left_offset_rad)
        self.head_tilt_cg = HeadTiltCommandGroup(self.node.head_tilt_calibrated_offset_rad,
                                                 self.node.head_tilt_calibrated_looking_up_offset_rad)
        self.wrist_yaw_cg = WristYawCommandGroup()
        self.gripper_cg = GripperCommandGroup()

    def execute_cb(self, goal):
        with self.node.robot_stop_lock:
            # Escape stopped mode to execute trajectory
            self.node.stop_the_robot = False
        self.node.robot_mode_rwlock.acquire_read()

        # For now, ignore goal time and configuration tolerances.
        commanded_joint_names = goal.trajectory.joint_names
        rospy.loginfo('{0} joint_traj action: \
                       New trajectory received with joint_names = {1}'.format(self.node.node_name,
                                                                              commanded_joint_names))

        ###################################################
        # Decide what to do based on the commanded joints.
        if self.node.use_lift:
            command_groups = [self.telescoping_cg, self.lift_cg, self.mobile_base_cg,
                              self.head_pan_cg, self.head_tilt_cg, self.wrist_yaw_cg, self.gripper_cg]
        else:
            command_groups = [self.telescoping_cg, self.mobile_base_cg, self.head_pan_cg,
                              self.head_tilt_cg, self.wrist_yaw_cg, self.gripper_cg]

        updates = [c.update(commanded_joint_names, self.invalid_joints_callback, self.node.robot_mode)
                   for c in command_groups]
        if not all(updates):
            # The joint names violated at least one of the command
            # group's requirements. The command group should have
            # reported the error.
            self.node.robot_mode_rwlock.release_read()
            return

        num_valid_points = sum([c.get_num_valid_commands() for c in command_groups])
        if num_valid_points <= 0:
            err_str = 'Received a command without any valid joint names. \
                       Received joint names = {0}'.format(commanded_joint_names)
            self.invalid_joints_callback(err_str)
            self.node.robot_mode_rwlock.release_read()
            return
        elif num_valid_points != len(commanded_joint_names):
            err_str = 'Received only {0} valid joints out of {1} total joints. \
                       Received joint names = {2}'.format(num_valid_points, len(commanded_joint_names),
                                                          commanded_joint_names)
            self.invalid_joints_callback(err_str)
            self.node.robot_mode_rwlock.release_read()
            return

        ###################################################
        # Try to reach each of the goals in sequence until
        # an error is detected or success is achieved.
        for pointi, point in enumerate(goal.trajectory.points):
            rospy.logdebug('{0} joint_traj action: \
                            target point #{1} = <{2}>'.format(self.node.node_name, pointi, point))

            valid_goals = [c.set_goal(point, self.invalid_goal_callback,
                                      self.node.mobile_base_manipulation_origin) for c in command_groups]
            if not all(valid_goals):
                # At least one of the goals violated the requirements
                # of a command group. Any violations should have been
                # reported as errors by the command groups.
                self.node.robot_mode_rwlock.release_read()
                return

            first_time = True
            incremental_commands_executed = False
            update_rate = rospy.Rate(15.0)
            goal_start_time = rospy.Time.now()

            while True:
                # Get copy of the current robot status (uses lock held by the robot).
                robot_status = self.node.robot.get_status()

                if first_time:
                    for c in command_groups:
                        c.init_execution(robot_status)
                    first_time = False

                if self.node.use_lift:
                    lift_error_m = self.lift_cg.update_execution(robot_status, self.node.backlash_state)
                extension_error_m = self.telescoping_cg.update_execution(robot_status,
                                                                         self.node.backlash_state)
                mobile_base_error_m, mobile_base_error_rad = \
                    self.mobile_base_cg.update_execution(robot_status, self.node.backlash_state)
                self.head_pan_cg.update_execution(robot_status, self.node.backlash_state)
                self.head_tilt_cg.update_execution(robot_status, self.node.backlash_state)
                self.wrist_yaw_cg.update_execution(robot_status, self.node.backlash_state)
                self.gripper_cg.update_execution(robot_status, self.node.backlash_state)

                # Check if a premption request has been received.
                with self.node.robot_stop_lock:
                    if self.node.stop_the_robot or self.server.is_preempt_requested():
                        rospy.logdebug('{0} joint_traj action: PREEMPTION REQUESTED, but not stopping \
                                        current motions to allow smooth interpolation between \
                                        old and new commands.'.format(self.node.node_name))
                        self.server.set_preempted()
                        self.node.stop_the_robot = False
                        self.node.robot_mode_rwlock.release_read()
                        return

                if not incremental_commands_executed:
                    translate = (mobile_base_error_m is not None)
                    rotate = (mobile_base_error_rad is not None)
                    if translate and rotate:
                        err_str = 'Simultaneous translation and rotation of the mobile base requested. \
                                   This is not allowed.'
                        self.invalid_goal_callback(err_str)
                        self.node.robot_mode_rwlock.release_read()
                        return
                    if translate:
                        self.node.robot.base.translate_by(mobile_base_error_m)
                    if rotate:
                        self.node.robot.base.rotate_by(mobile_base_error_rad)

                    if self.telescoping_cg.extension_goal:
                        self.node.robot.arm.move_by(extension_error_m)
                        if extension_error_m < 0.0:
                            self.node.backlash_state['wrist_extension_retracted'] = True
                        else:
                            self.node.backlash_state['wrist_extension_retracted'] = False

                    if self.node.use_lift:
                        if self.lift_cg.lift_goal:
                            self.node.robot.lift.move_by(lift_error_m)

                    if self.head_pan_cg.joint_goal:
                        self.node.robot.head.move_by('head_pan', self.head_pan_cg.joint_error)
                        if self.head_pan_cg.joint_error > 0.0:
                            self.node.backlash_state['head_pan_looked_left'] = True
                        else:
                            self.node.backlash_state['head_pan_looked_left'] = False

                    if self.head_tilt_cg.joint_goal:
                        self.node.robot.head.move_by('head_tilt', self.head_tilt_cg.joint_error)
                        if self.head_tilt_cg.joint_target > (self.node.head_tilt_backlash_transition_angle_rad + self.node.head_tilt_calibrated_offset_rad):
                            self.node.backlash_state['head_tilt_looking_up'] = True
                        else:
                            self.node.backlash_state['head_tilt_looking_up'] = False

                    if self.wrist_yaw_cg.joint_goal:
                        self.node.robot.end_of_arm.move_to('wrist_yaw', self.wrist_yaw_cg.joint_target)

                    if self.gripper_cg.gripper_joint_goal:
                        gripper_command = self.gripper_cg.goal_gripper_joint
                        rospy.logdebug('{0} gripper debug: move_to stretch_gripper = \
                                        {1}'.format(self.node.node_name, gripper_command))
                        self.node.robot.end_of_arm.move_to('stretch_gripper', gripper_command)

                    self.node.robot.push_command()
                    incremental_commands_executed = True

                # Check if the goal positions have been reached.
                goals_reached = [c.goal_reached() for c in command_groups]
                if all(goals_reached):
                    if self.node.trajectory_debug:
                        rospy.loginfo('achieved goal!')
                    break

                if (rospy.Time.now() - goal_start_time) > self.node.default_goal_timeout_duration:
                    err_str = 'Time to execute the current goal point = <{0}> exceeded the \
                               default_goal_timeout = {1}'.format(point, self.node.default_goal_timeout_s)
                    self.goal_tolerance_violated_callback(err_str)
                    self.node.robot_mode_rwlock.release_read()
                    return

                update_rate.sleep()

            rospy.logdebug('{0} joint_traj action: Achieved target point.'.format(self.node.node_name))
            # Currently not providing feedback.

        self.success_callback()
        self.node.robot_mode_rwlock.release_read()
        return

    def invalid_joints_callback(self, err_str):
        rospy.logerr('{0} joint_traj action: {1}'.format(self.node.node_name, err_str))
        self.result.error_code = self.result.INVALID_JOINTS
        self.result.error_string = err_str
        self.server.set_aborted(self.result)

    def invalid_goal_callback(self, err_str):
        rospy.logerr('{0} joint_traj action: {1}'.format(self.node.node_name, err_str))
        self.result.error_code = self.result.INVALID_GOAL
        self.result.error_string = err_str
        self.result.set_aborted(self.result)

    def goal_tolerance_violated_callback(self, err_str):
        rospy.logerr('{0} joint_traj action: {1}'.format(self.node.node_name, err_str))
        self.result.error_code = self.result.GOAL_TOLERANCE_VIOLATED
        self.result.error_string = err_str
        self.server.set_aborted(self.result)

    def success_callback(self):
        rospy.loginfo('{0} joint_traj action: Achieved all target points!'.format(self.node.node_name))
        self.result.error_code = self.result.SUCCESSFUL
        self.result.error_string = "Achieved all target points!"
        self.server.set_succeeded(self.result)
