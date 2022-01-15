#! /usr/bin/env python3

import importlib

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint

from command_groups import HeadPanCommandGroup, HeadTiltCommandGroup, \
                           WristYawCommandGroup, GripperCommandGroup, \
                           ArmCommandGroup, LiftCommandGroup, \
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

        self.head_pan_cg = HeadPanCommandGroup(node=self.node) \
            if 'head_pan' in self.node.robot.head.joints else None
        self.head_tilt_cg = HeadTiltCommandGroup(node=self.node) \
            if 'head_tilt' in self.node.robot.head.joints else None
        self.wrist_yaw_cg = WristYawCommandGroup(node=self.node) \
            if 'wrist_yaw' in self.node.robot.end_of_arm.joints else None
        self.gripper_cg = GripperCommandGroup(node=self.node) \
            if 'stretch_gripper' in self.node.robot.end_of_arm.joints else None
        self.arm_cg = ArmCommandGroup(node=self.node)
        self.lift_cg = LiftCommandGroup(node=self.node)
        self.mobile_base_cg = MobileBaseCommandGroup(node=self.node)
        self.command_groups = [self.arm_cg, self.lift_cg, self.mobile_base_cg, self.head_pan_cg,
                               self.head_tilt_cg, self.wrist_yaw_cg, self.gripper_cg]
        self.command_groups = [cg for cg in self.command_groups if cg is not None]

        for joint in self.node.robot.end_of_arm.joints:
            module_name = self.node.robot.end_of_arm.params['devices'][joint].get('ros_py_module_name')
            class_name = self.node.robot.end_of_arm.params['devices'][joint].get('ros_py_class_name')
            if module_name and class_name:
                endofarm_cg = getattr(importlib.import_module(module_name), class_name)(node=self.node)
                self.command_groups.append(endofarm_cg)

    def execute_cb(self, goal):
        with self.node.robot_stop_lock:
            # Escape stopped mode to execute trajectory
            self.node.stop_the_robot = False
        self.node.robot_mode_rwlock.acquire_read()

        # For now, ignore goal time and configuration tolerances.
        commanded_joint_names = goal.trajectory.joint_names
        rospy.loginfo(("{0} joint_traj action: New trajectory received with joint_names = "
                       "{1}").format(self.node.node_name, commanded_joint_names))

        ###################################################
        # Decide what to do based on the commanded joints.
        updates = [c.update(commanded_joint_names, self.invalid_joints_callback,
                   robot_mode=self.node.robot_mode)
                   for c in self.command_groups]
        if not all(updates):
            # The joint names violated at least one of the command
            # group's requirements. The command group should have
            # reported the error.
            self.node.robot_mode_rwlock.release_read()
            return

        num_valid_points = sum([c.get_num_valid_commands() for c in self.command_groups])
        if num_valid_points <= 0:
            err_str = ("Received a command without any valid joint names."
                       "Received joint names = {0}").format(commanded_joint_names)
            self.invalid_joints_callback(err_str)
            self.node.robot_mode_rwlock.release_read()
            return
        elif num_valid_points != len(commanded_joint_names):
            err_str = ("Received only {0} valid joints out of {1} total joints. Received joint names = "
                       "{2}").format(num_valid_points, len(commanded_joint_names), commanded_joint_names)
            self.invalid_joints_callback(err_str)
            self.node.robot_mode_rwlock.release_read()
            return

        ###################################################
        # Try to reach each of the goals in sequence until
        # an error is detected or success is achieved.
        for pointi, point in enumerate(goal.trajectory.points):
            rospy.logdebug(("{0} joint_traj action: "
                            "target point #{1} = <{2}>").format(self.node.node_name, pointi, point))

            valid_goals = [c.set_goal(point, self.invalid_goal_callback, self.node.fail_out_of_range_goal)
                           for c in self.command_groups]
            if not all(valid_goals):
                # At least one of the goals violated the requirements
                # of a command group. Any violations should have been
                # reported as errors by the command groups.
                self.node.robot_mode_rwlock.release_read()
                return

            robot_status = self.node.robot.get_status() # uses lock held by robot
            for c in self.command_groups:
                c.init_execution(self.node.robot, robot_status)
            self.node.robot.push_command()

            goals_reached = [c.goal_reached() for c in self.command_groups]
            update_rate = rospy.Rate(15.0)
            goal_start_time = rospy.Time.now()

            while not all(goals_reached):
                if (rospy.Time.now() - goal_start_time) > self.node.default_goal_timeout_duration:
                    err_str = ("Time to execute the current goal point = <{0}> exceeded the "
                               "default_goal_timeout = {1}").format(point, self.node.default_goal_timeout_s)
                    self.goal_tolerance_violated_callback(err_str)
                    self.node.robot_mode_rwlock.release_read()
                    return

                # Check if a premption request has been received.
                with self.node.robot_stop_lock:
                    if self.node.stop_the_robot or self.server.is_preempt_requested():
                        rospy.logdebug(("{0} joint_traj action: PREEMPTION REQUESTED, but not stopping "
                                        "current motions to allow smooth interpolation between "
                                        "old and new commands.").format(self.node.node_name))
                        self.server.set_preempted()
                        self.node.stop_the_robot = False
                        self.node.robot_mode_rwlock.release_read()
                        return

                robot_status = self.node.robot.get_status()
                named_errors = [c.update_execution(robot_status, success_callback=self.success_callback)
                                for c in self.command_groups]
                # It's not clear how this could ever happen. The
                # groups in command_groups.py seem to return
                # (self.name, self.error) or None, rather than True.
                if any(ret == True for ret in named_errors):
                    self.node.robot_mode_rwlock.release_read()
                    return

                self.feedback_callback(commanded_joint_names, point, named_errors)
                goals_reached = [c.goal_reached() for c in self.command_groups]
                update_rate.sleep()

            rospy.logdebug("{0} joint_traj action: Achieved target point.".format(self.node.node_name))

        self.success_callback("Achieved all target points.")
        self.node.robot_mode_rwlock.release_read()
        return

    def invalid_joints_callback(self, err_str):
        if self.server.is_active() or self.server.is_preempt_requested():
            rospy.logerr("{0} joint_traj action: {1}".format(self.node.node_name, err_str))
            self.result.error_code = self.result.INVALID_JOINTS
            self.result.error_string = err_str
            self.server.set_aborted(self.result)

    def invalid_goal_callback(self, err_str):
        if self.server.is_active() or self.server.is_preempt_requested():
            rospy.logerr("{0} joint_traj action: {1}".format(self.node.node_name, err_str))
            self.result.error_code = self.result.INVALID_GOAL
            self.result.error_string = err_str
            self.server.set_aborted(self.result)

    def goal_tolerance_violated_callback(self, err_str):
        if self.server.is_active() or self.server.is_preempt_requested():
            rospy.logerr("{0} joint_traj action: {1}".format(self.node.node_name, err_str))
            self.result.error_code = self.result.GOAL_TOLERANCE_VIOLATED
            self.result.error_string = err_str
            self.server.set_aborted(self.result)

    def feedback_callback(self, commanded_joint_names, desired_point, named_errors):
        clean_named_errors = []
        for named_error in named_errors:
            if type(named_error) == tuple:
                clean_named_errors.append(named_error)
            elif type(named_error) == list:
                clean_named_errors += named_error
        clean_named_errors_dict = dict((k, v) for k, v in clean_named_errors)

        actual_point = JointTrajectoryPoint()
        error_point = JointTrajectoryPoint()
        for i, commanded_joint_name in enumerate(commanded_joint_names):
            error_point.positions.append(clean_named_errors_dict[commanded_joint_name])
            actual_point.positions.append(desired_point.positions[i] - clean_named_errors_dict[commanded_joint_name])

        rospy.logdebug("{0} joint_traj action: sending feedback".format(self.node.node_name))
        self.feedback.header.stamp = rospy.Time.now()
        self.feedback.joint_names = commanded_joint_names
        self.feedback.desired = desired_point
        self.feedback.actual = actual_point
        self.feedback.error = error_point
        self.server.publish_feedback(self.feedback)

    def success_callback(self, success_str):
        rospy.loginfo("{0} joint_traj action: {1}".format(self.node.node_name, success_str))
        self.result.error_code = self.result.SUCCESSFUL
        self.result.error_string = success_str
        self.server.set_succeeded(self.result)
