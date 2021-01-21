#! /usr/bin/env python
from __future__ import print_function

import numpy as np
import hello_helpers.hello_misc as hm
from hello_helpers.gripper_conversion import GripperConversion


class SimpleCommandGroup:
    def __init__(self, joint_name, joint_range, acceptable_joint_error=0.015):
        """Simple command group to extend

        Attributes
        ----------
        name: str
            joint name
        range: tuple(float)
            acceptable joint bounds
        active: bool
            whether joint is active
        index: int
            index of joint's goal in point
        goal: dict
            components of the goal
        error: float
            the error between actual and desired
        acceptable_joint_error: float
            how close to zero the error must reach
        """
        self.name = joint_name
        self.range = joint_range
        self.active = False
        self.index = None
        self.goal = {"position": None}
        self.error = None
        self.acceptable_joint_error = acceptable_joint_error

    def get_num_valid_commands(self):
        """Returns number of active joints in the group

        Returns
        -------
        int
            the number of active joints within this group
        """
        if self.active:
            return 1

        return 0

    def update(self, commanded_joint_names, invalid_joints_callback, **kwargs):
        """Activates joints in the group

        Checks commanded joints to activate the command
        group and validates joints used correctly.

        Parameters
        ----------
        commanded_joint_names: list(str)
            list of commanded joints in the trajectory
        invalid_joints_callback: func
            error callback for misuse of joints in trajectory

        Returns
        -------
        bool
            False if commanded joints invalid, else True
        """
        self.active = False
        self.index = None
        if self.name in commanded_joint_names:
            self.index = commanded_joint_names.index(self.name)
            self.active = True

        return True

    def set_goal(self, point, invalid_goal_callback, fail_out_of_range_goal, **kwargs):
        """Sets goal for the joint group

        Sets and validates the goal point for the joints
        in this command group.

        Parameters
        ----------
        point: trajectory_msgs.JointTrajectoryPoint
            the target point for all joints
        invalid_goal_callback: func
            error callback for invalid goal
        fail_out_of_range_goal: bool
            whether to bound out-of-range goals to range or fail

        Returns
        -------
        bool
            False if commanded goal invalid, else True
        """
        self.goal = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        if self.active:
            goal_pos = point.positions[self.index] if len(point.positions) > self.index else None
            if goal_pos is None:
                err_str = ("Received goal point with positions array length={0}. "
                           "This joint ({1})'s index is {2}. Length of array must cover all joints listed "
                           "in commanded_joint_names.").format(len(point.positions), self.name, self.index)
                invalid_goal_callback(err_str)
                return False

            self.goal['position'] = hm.bound_ros_command(self.range, goal_pos, fail_out_of_range_goal)
            self.goal['velocity'] = point.velocities[self.index] if len(point.velocities) > self.index else None
            self.goal['acceleration'] = point.accelerations[self.index] if len(point.accelerations) > self.index else None
            self.goal['contact_threshold'] = point.effort[self.index] if len(point.effort) > self.index else None
            if self.goal['position'] is None:
                err_str = ("Received {0} goal point that is out of bounds. "
                            "Range = {1}, but goal point = {2}.").format(self.name, self.range, goal_pos)
                invalid_goal_callback(err_str)
                return False

        return True

    def init_execution(self, robot, robot_status, **kwargs):
        """Starts execution of the point

        Uses Stretch's Python API to begin moving to the
        target point.

        Parameters
        ----------
        robot: stretch_body.robot.Robot
            top-level interface to Python API
        robot_status: dict
            robot's current status
        """
        raise NotImplementedError

    def update_execution(self, robot_status, **kwargs):
        """Monitors progress of joint group

        Checks against robot's status to track progress
        towards the target point.

        This method must set self.error.

        Parameters
        ----------
        robot_status: dict
            robot's current status

        Returns
        -------
        float/None
            error value if group active, else None
        """
        raise NotImplementedError

    def goal_reached(self):
        """Returns whether reached target point

        Returns
        -------
        bool
            if active, whether reached target point, else True
        """
        if self.active:
            return (abs(self.error) < self.acceptable_joint_error)

        return True


class HeadPanCommandGroup(SimpleCommandGroup):
    def __init__(self, range_rad, head_pan_calibrated_offset, head_pan_calibrated_looked_left_offset):
        SimpleCommandGroup.__init__(self, 'joint_head_pan', range_rad, acceptable_joint_error=0.15)
        self.head_pan_calibrated_offset = head_pan_calibrated_offset
        self.head_pan_calibrated_looked_left_offset = head_pan_calibrated_looked_left_offset

    def set_trajectory_goals(self, points, robot):
        if self.active:
            for waypoint in points:
                t = waypoint.time_from_start.to_sec()
                x = None
                if len(waypoint.positions) > self.index:
                    x = waypoint.positions[self.index]
                v = None
                if len(waypoint.velocities) > self.index:
                    v = waypoint.velocities[self.index]
                a = None
                if len(waypoint.accelerations) > self.index:
                    a = waypoint.accelerations[self.index]
                robot.head.get_joint('head_pan').trajectory.add_waypoint(t_s=t, x_r=x, v_r=v, a_r=a)

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            _, pan_error = self.update_execution(robot_status, backlash_state=kwargs['backlash_state'])
            robot.head.move_by('head_pan', pan_error, v_r=self.goal['velocity'], a_r=self.goal['acceleration'])
            if pan_error > 0.0:
                kwargs['backlash_state']['head_pan_looked_left'] = True
            else:
                kwargs['backlash_state']['head_pan_looked_left'] = False

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        backlash_state = kwargs['backlash_state']
        if self.active:
            if backlash_state['head_pan_looked_left']:
                pan_backlash_correction = self.head_pan_calibrated_looked_left_offset
            else:
                pan_backlash_correction = 0.0
            pan_current = robot_status['head']['head_pan']['pos'] + \
                          self.head_pan_calibrated_offset + pan_backlash_correction
            self.error = self.goal['position'] - pan_current
            return self.name, self.error

        return None


class HeadTiltCommandGroup(SimpleCommandGroup):
    def __init__(self, range_rad, head_tilt_calibrated_offset,
                 head_tilt_calibrated_looking_up_offset,
                 head_tilt_backlash_transition_angle):
        SimpleCommandGroup.__init__(self, 'joint_head_tilt', range_rad, acceptable_joint_error=0.52)
        self.head_tilt_calibrated_offset = head_tilt_calibrated_offset
        self.head_tilt_calibrated_looking_up_offset = head_tilt_calibrated_looking_up_offset
        self.head_tilt_backlash_transition_angle = head_tilt_backlash_transition_angle

    def set_trajectory_goals(self, points, robot):
        if self.active:
            for waypoint in points:
                t = waypoint.time_from_start.to_sec()
                x = None
                if len(waypoint.positions) > self.index:
                    x = waypoint.positions[self.index]
                v = None
                if len(waypoint.velocities) > self.index:
                    v = waypoint.velocities[self.index]
                a = None
                if len(waypoint.accelerations) > self.index:
                    a = waypoint.accelerations[self.index]
                robot.head.get_joint('head_tilt').trajectory.add_waypoint(t_s=t, x_r=x, v_r=v, a_r=a)

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            _, tilt_error = self.update_execution(robot_status, backlash_state=kwargs['backlash_state'])
            robot.head.move_by('head_tilt', tilt_error, v_r=self.goal['velocity'], a_r=self.goal['acceleration'])
            if tilt_error > (self.head_tilt_backlash_transition_angle + self.head_tilt_calibrated_offset):
                kwargs['backlash_state']['head_tilt_looking_up'] = True
            else:
                kwargs['backlash_state']['head_tilt_looking_up'] = False

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        backlash_state = kwargs['backlash_state']
        if self.active:
            if backlash_state['head_tilt_looking_up']:
                tilt_backlash_correction = self.head_tilt_calibrated_looking_up_offset
            else:
                tilt_backlash_correction = 0.0
            tilt_current = robot_status['head']['head_tilt']['pos'] + \
                           self.head_tilt_calibrated_offset + tilt_backlash_correction
            self.error = self.goal['position'] - tilt_current
            return self.name, self.error

        return None


class WristYawCommandGroup(SimpleCommandGroup):
    def __init__(self, range_rad):
        SimpleCommandGroup.__init__(self, 'joint_wrist_yaw', range_rad)

    def set_trajectory_goals(self, points, robot):
        if self.active:
            for waypoint in points:
                t = waypoint.time_from_start.to_sec()
                x = None
                if len(waypoint.positions) > self.index:
                    x = waypoint.positions[self.index]
                v = None
                if len(waypoint.velocities) > self.index:
                    v = waypoint.velocities[self.index]
                a = None
                if len(waypoint.accelerations) > self.index:
                    a = waypoint.accelerations[self.index]
                robot.end_of_arm.motors['wrist_yaw'].trajectory.add_waypoint(t_s=t, x_r=x, v_r=v, a_r=a)

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            robot.end_of_arm.move_by('wrist_yaw',
                                     self.update_execution(robot_status)[1],
                                     v_r=self.goal['velocity'],
                                     a_r=self.goal['acceleration'])

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        if self.active:
            self.error = self.goal['position'] - robot_status['end_of_arm']['wrist_yaw']['pos']
            return self.name, self.error

        return None


class GripperCommandGroup(SimpleCommandGroup):
    def __init__(self, range_robotis):
        SimpleCommandGroup.__init__(self, None, None, acceptable_joint_error=1.0)
        self.gripper_joint_names = ['joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture']
        self.gripper_conversion = GripperConversion()
        self.range_aperture_m = (self.gripper_conversion.robotis_to_aperture(range_robotis[0]),
                                 self.gripper_conversion.robotis_to_aperture(range_robotis[1]))
        self.range_finger_rad = (self.gripper_conversion.robotis_to_finger(range_robotis[0]),
                                 self.gripper_conversion.robotis_to_finger(range_robotis[1]))

    def update(self, commanded_joint_names, invalid_joints_callback, **kwargs):
        self.active = False
        self.index = None
        active_gripper_joint_names = list(set(commanded_joint_names) & set(self.gripper_joint_names))
        if len(active_gripper_joint_names) > 1:
            err_str = ("Received a command for the gripper that includes more than one gripper joint name: {0}. "
                       "Only one joint name is allowed to be used for a gripper command to avoid conflicts "
                       "and confusion. The gripper only has a single degree of freedom that can be "
                       "controlled using the following three mutually exclusive joint names: "
                       "{1}.").format(active_gripper_joint_names, self.gripper_joint_names)
            invalid_joints_callback(err_str)
            return False
        elif len(active_gripper_joint_names) == 1:
            self.name = active_gripper_joint_names[0]
            self.index = commanded_joint_names.index(self.name)
            self.active = True

        return True

    def set_goal(self, point, invalid_goal_callback, fail_out_of_range_goal, **kwargs):
        self.goal = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        if self.active:
            goal_pos = point.positions[self.index] if len(point.positions) > self.index else None
            if goal_pos is None:
                err_str = ("Received goal point with positions array length={0}. "
                           "This joint ({1})'s index is {2}. Length of array must cover all joints listed "
                           "in commanded_joint_names.").format(len(point.positions), self.name, self.index)
                invalid_goal_callback(err_str)
                return False

            joint_range = self.range_aperture_m if (self.name == 'gripper_aperture') else self.range_finger_rad
            self.goal['position'] = hm.bound_ros_command(joint_range, goal_pos, fail_out_of_range_goal)
            self.goal['velocity'] = point.velocities[self.index] if len(point.velocities) > self.index else None
            self.goal['acceleration'] = point.accelerations[self.index] if len(point.accelerations) > self.index else None
            self.goal['contact_threshold'] = point.effort[self.index] if len(point.effort) > self.index else None
            if self.goal['position'] is None:
                err_str = ("Received {0} goal point that is out of bounds. "
                            "Range = {1}, but goal point = {2}.").format(self.name, joint_range, goal_pos)
                invalid_goal_callback(err_str)
                return False

        return True

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            _, gripper_error = self.update_execution(robot_status)
            if (self.name == 'gripper_aperture'):
                gripper_robotis_error = self.gripper_conversion.aperture_to_robotis(gripper_error)
            elif (self.name == 'joint_gripper_finger_left') or (self.name == 'joint_gripper_finger_right'):
                gripper_robotis_error = self.gripper_conversion.finger_to_robotis(gripper_error)
            robot.end_of_arm.move_by('stretch_gripper',
                                     gripper_robotis_error,
                                     v_r=self.goal['velocity'],
                                     a_r=self.goal['acceleration'])

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        if self.active:
            robotis_pct = robot_status['end_of_arm']['stretch_gripper']['pos_pct']
            if (self.name == 'gripper_aperture'):
                gripper_current = self.gripper_conversion.robotis_to_aperture(robotis_pct)
            elif (self.name == 'joint_gripper_finger_left') or (self.name == 'joint_gripper_finger_right'):
                gripper_current = self.gripper_conversion.robotis_to_finger(robotis_pct)

            self.error = self.goal['position'] - gripper_current
            return self.name, self.error

        return None


class TelescopingCommandGroup(SimpleCommandGroup):
    def __init__(self, range_m, wrist_extension_calibrated_retracted_offset):
        #SimpleCommandGroup.__init__(self, 'wrist_extension', range_m, acceptable_joint_error=0.005)
        SimpleCommandGroup.__init__(self, 'wrist_extension', range_m, acceptable_joint_error=0.008)
        self.wrist_extension_calibrated_retracted_offset = wrist_extension_calibrated_retracted_offset
        self.telescoping_joints = ['joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0']
        self.is_telescoping = False

    def get_num_valid_commands(self):
        if self.active and self.is_telescoping:
            return len(self.telescoping_joints)
        elif self.active:
            return 1

        return 0

    def update(self, commanded_joint_names, invalid_joints_callback, **kwargs):
        self.active = False
        self.is_telescoping = False
        self.index = None
        active_telescoping_joint_names = list(set(commanded_joint_names) & set(self.telescoping_joints))
        if self.name in commanded_joint_names:
            if len(active_telescoping_joint_names) == 0:
                self.index = commanded_joint_names.index(self.name)
                self.active = True
            else:
                err_str = ("Received a command for the wrist_extension joint and one or more telescoping_joints. "
                           "These are mutually exclusive options. The joint names in the received command = "
                           "{0}").format(commanded_joint_names)
                invalid_joints_callback(err_str)
                return False
        elif len(active_telescoping_joint_names) != 0:
            if len(active_telescoping_joint_names) == len(self.telescoping_joints):
                self.active = True
                self.is_telescoping = True
                self.index = [commanded_joint_names.index(i) for i in self.telescoping_joints]
            else:
                err_str = ("Commands with telescoping joints requires all telescoping joints to be present. "
                           "Only received {0} of {1} telescoping joints. They are = "
                           "{2}").format(len(active_telescoping_joint_names), len(self.telescoping_joints),
                                         active_telescoping_joint_names)
                invalid_joints_callback(err_str)
                return False

        return True

    def set_trajectory_goals(self, points, robot):
        if self.active:
            for waypoint in points:
                t = waypoint.time_from_start.to_sec()
                x = None
                if len(waypoint.positions) > self.index:
                    x = waypoint.positions[self.index]
                v = None
                if len(waypoint.velocities) > self.index:
                    v = waypoint.velocities[self.index]
                a = None
                if len(waypoint.accelerations) > self.index:
                    a = waypoint.accelerations[self.index]
                robot.arm.trajectory.add_waypoint(t_s=t, x_m=x, v_m=v, a_m=a)

    def set_goal(self, point, invalid_goal_callback, fail_out_of_range_goal, **kwargs):
        self.goal = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        if self.active:
            if self.is_telescoping:
                goal_pos = sum([point.positions[i] for i in self.index]) \
                           if len(point.positions) > max(self.index) else None
                self.goal['velocity'] = point.velocities[self.index[0]] \
                                        if len(point.velocities) > self.index[0] else None
                self.goal['acceleration'] = point.accelerations[self.index[0]] \
                                            if len(point.accelerations) > self.index[0] else None
                self.goal['contact_threshold'] = point.effort[self.index[0]] \
                                                 if len(point.effort) > self.index[0] else None
            else:
                goal_pos = point.positions[self.index] \
                           if len(point.positions) > self.index else None
                self.goal['velocity'] = point.velocities[self.index] \
                                        if len(point.velocities) > self.index else None
                self.goal['acceleration'] = point.accelerations[self.index] \
                                            if len(point.accelerations) > self.index else None
                self.goal['contact_threshold'] = point.effort[self.index] \
                                                 if len(point.effort) > self.index else None

            if goal_pos is None:
                err_str = ("Received goal point with positions array length={0}. "
                           "This joint ({1})'s index is {2}. Length of array must cover all joints listed "
                           "in commanded_joint_names.").format(len(point.positions), self.name, self.index)
                invalid_goal_callback(err_str)
                return False

            self.goal['position'] = hm.bound_ros_command(self.range, goal_pos, fail_out_of_range_goal)
            if self.goal['position'] is None:
                err_str = ("Received {0} goal point that is out of bounds. "
                            "Range = {1}, but goal point = {2}.").format(self.name, self.range, goal_pos)
                invalid_goal_callback(err_str)
                return False

        return True

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            _, extension_error_m = self.update_execution(robot_status, backlash_state=kwargs['backlash_state'])
            robot.arm.move_by(extension_error_m,
                              v_m=self.goal['velocity'],
                              a_m=self.goal['acceleration'],
                              contact_thresh_pos_N=self.goal['contact_threshold'],
                              contact_thresh_neg_N=-self.goal['contact_threshold'] \
                                                   if self.goal['contact_threshold'] is not None else None)
            if extension_error_m < 0.0:
                kwargs['backlash_state']['wrist_extension_retracted'] = True
            else:
                kwargs['backlash_state']['wrist_extension_retracted'] = False

    def update_execution(self, robot_status, **kwargs):
        backlash_state = kwargs['backlash_state']
        success_callback = kwargs['success_callback'] if 'success_callback' in kwargs.keys() else None
        self.error = None
        if self.active:
            if success_callback and robot_status['arm']['motor']['in_guarded_event']:
                success_callback("{0} contact detected.".format(self.name))
                return True
            if backlash_state['wrist_extension_retracted']:
                arm_backlash_correction = self.wrist_extension_calibrated_retracted_offset
            else:
                arm_backlash_correction = 0.0
            extension_current = robot_status['arm']['pos'] + arm_backlash_correction
            self.error = self.goal['position'] - extension_current
            return (self.telescoping_joints, self.error) if self.is_telescoping else (self.name, self.error)

        return None


class LiftCommandGroup(SimpleCommandGroup):
    def __init__(self, range_m):
        SimpleCommandGroup.__init__(self, 'joint_lift', range_m)

    def set_trajectory_goals(self, points, robot):
        if self.active:
            for waypoint in points:
                t = waypoint.time_from_start.to_sec()
                x = None
                if len(waypoint.positions) > self.index:
                    x = waypoint.positions[self.index]
                v = None
                if len(waypoint.velocities) > self.index:
                    v = waypoint.velocities[self.index]
                a = None
                if len(waypoint.accelerations) > self.index:
                    a = waypoint.accelerations[self.index]
                robot.lift.trajectory.add_waypoint(t_s=t, x_m=x, v_m=v, a_m=a)

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            robot.lift.move_by(self.update_execution(robot_status)[1],
                               v_m=self.goal['velocity'],
                               a_m=self.goal['acceleration'],
                               contact_thresh_pos_N=self.goal['contact_threshold'],
                               contact_thresh_neg_N=-self.goal['contact_threshold'] \
                                                    if self.goal['contact_threshold'] is not None else None)

    def update_execution(self, robot_status, **kwargs):
        success_callback = kwargs['success_callback'] if 'success_callback' in kwargs.keys() else None
        self.error = None
        if self.active:
            if success_callback and robot_status['lift']['motor']['in_guarded_event']:
                success_callback("{0} contact detected.".format(self.name))
                return True
            self.error = self.goal['position'] - robot_status['lift']['pos']
            return self.name, self.error

        return None


class MobileBaseCommandGroup(SimpleCommandGroup):
    def __init__(self, virtual_range_m=(-0.5, 0.5)):
        SimpleCommandGroup.__init__(self, 'joint_mobile_base_translation', virtual_range_m,
                                    acceptable_joint_error=0.005)
        self.incrementing_joint_names = ['translate_mobile_base', 'rotate_mobile_base']
        self.active_translate_mobile_base = False
        self.active_rotate_mobile_base = False
        self.acceptable_mobile_base_error_m = 0.005
        self.excellent_mobile_base_error_m = 0.005
        self.acceptable_mobile_base_error_rad = (np.pi/180.0) * 6.0
        self.excellent_mobile_base_error_rad = (np.pi/180.0) * 0.6
        self.min_m_per_s = 0.002
        self.min_rad_per_s = np.radians(1.0)

    def get_num_valid_commands(self):
        if self.active:
            num_inc = self.active_translate_mobile_base + self.active_rotate_mobile_base
            return num_inc if num_inc > 0 else 1

        return 0

    def update(self, commanded_joint_names, invalid_joints_callback, **kwargs):
        robot_mode = kwargs['robot_mode']
        self.active = False
        self.active_translate_mobile_base = False
        self.active_rotate_mobile_base = False
        self.index = None
        self.index_translate_mobile_base = None
        self.index_rotate_mobile_base = None
        active_incrementing_joint_names = list(set(commanded_joint_names) & set(self.incrementing_joint_names))

        if self.name in commanded_joint_names:
            if robot_mode == 'manipulation':
                if len(active_incrementing_joint_names) == 0:
                    self.active = True
                    self.index = commanded_joint_names.index(self.name)
                else:
                    err_str = ("Received a command for the mobile base virtual joint ({0}}) "
                               "and mobile base incremental motions ({1}). These are "
                               "mutually exclusive options. The joint names in the received command = "
                               "{2}").format(self.name, active_incrementing_joint_names, commanded_joint_names)
                    invalid_joints_callback(err_str)
                    return False
            else:
                err_str = ("Must be in manipulation mode to receive a command for the "
                           "{0} joint. Current mode = {1}.").format(self.name, robot_mode)
                invalid_joints_callback(err_str)
                return False
        elif len(active_incrementing_joint_names) != 0:
            if robot_mode == 'position':
                self.active = True
                if 'translate_mobile_base' in active_incrementing_joint_names:
                    self.active_translate_mobile_base = True
                    self.index_translate_mobile_base = commanded_joint_names.index('translate_mobile_base')
                if 'rotate_mobile_base' in active_incrementing_joint_names:
                    self.active_rotate_mobile_base = True
                    self.index_rotate_mobile_base = commanded_joint_names.index('rotate_mobile_base')
            elif robot_mode == 'experimental_manipulation':
                self.active = True
                if 'translate_mobile_base' in active_incrementing_joint_names:
                    self.active_translate_mobile_base = True
                    self.index_translate_mobile_base = commanded_joint_names.index('translate_mobile_base')
                if 'rotate_mobile_base' in active_incrementing_joint_names:
                    self.active_rotate_mobile_base = True
                    self.index_rotate_mobile_base = commanded_joint_names.index('rotate_mobile_base')
                if self.active_translate_mobile_base and self.active_rotate_mobile_base:
                    err_str = ("Experimental manipulation mode cannot receive both mobile base translate and rotate joints")
                    invalid_joints_callback(err_str)
            else:
                err_str = ("Must be in position mode to receive a command for the {0} joint(s). "
                           "Current mode = {1}.").format(active_positioning_joint_names, robot_mode)
                invalid_joints_callback(err_str)
                return False

        return True

    def set_trajectory_goals(self, points, robot):
        if self.active_translate_mobile_base:
            for waypoint in points:
                t = waypoint.time_from_start.to_sec()
                x = None
                if len(waypoint.positions) > self.index_translate_mobile_base:
                    x = waypoint.positions[self.index_translate_mobile_base]
                v = None
                if len(waypoint.velocities) > self.index_translate_mobile_base:
                    v = waypoint.velocities[self.index_translate_mobile_base]
                a = None
                if len(waypoint.accelerations) > self.index_translate_mobile_base:
                    a = waypoint.accelerations[self.index_translate_mobile_base]
                robot.base.trajectory.add_translate_waypoint(t_s=t, x_m=x, v_m=v, a_m=a)
        elif self.active_rotate_mobile_base:
            for waypoint in points:
                t = waypoint.time_from_start.to_sec()
                x = None
                if len(waypoint.positions) > self.index_rotate_mobile_base:
                    x = waypoint.positions[self.index_rotate_mobile_base]
                v = None
                if len(waypoint.velocities) > self.index_rotate_mobile_base:
                    v = waypoint.velocities[self.index_rotate_mobile_base]
                a = None
                if len(waypoint.accelerations) > self.index_rotate_mobile_base:
                    a = waypoint.accelerations[self.index_rotate_mobile_base]
                robot.base.trajectory.add_rotate_waypoint(t_s=t, x_r=x, v_r=v, a_r=a)

    def set_goal(self, point, invalid_goal_callback, fail_out_of_range_goal, **kwargs):
        self.goal = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        self.goal_translate_mobile_base = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        self.goal_rotate_mobile_base = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        if self.active:
            if self.active_translate_mobile_base or self.active_rotate_mobile_base:
                if len(point.positions) <= self.index_translate_mobile_base and len(point.positions) <= self.index_rotate_mobile_base:
                    err_str = ("Received goal point with positions array length={0}. These joints ({1})'s "
                               "indices are {2} & {3} respectively. Length of array must cover all joints "
                               "listed in commanded_joint_names.").format(len(point.positions),
                                                                          self.incrementing_joint_names,
                                                                          self.index_translate_mobile_base,
                                                                          self.index_rotate_mobile_base)
                    invalid_goal_callback(err_str)
                    return False

                if self.active_translate_mobile_base and \
                   not np.isclose(point.positions[self.index_translate_mobile_base], 0.0, rtol=1e-5, atol=1e-8, equal_nan=False):
                    self.goal_translate_mobile_base['position'] = point.positions[self.index_translate_mobile_base]
                    self.goal_translate_mobile_base['velocity'] = point.velocities[self.index_translate_mobile_base] if len(point.velocities) > self.index_translate_mobile_base else None
                    self.goal_translate_mobile_base['acceleration'] = point.accelerations[self.index_translate_mobile_base] if len(point.accelerations) > self.index_translate_mobile_base else None
                    self.goal_translate_mobile_base['contact_threshold'] = point.effort[self.index_translate_mobile_base] if len(point.effort) > self.index_translate_mobile_base else None

                if self.active_rotate_mobile_base and \
                   not np.isclose(point.positions[self.index_rotate_mobile_base], 0.0, rtol=1e-5, atol=1e-8, equal_nan=False):
                    self.goal_rotate_mobile_base['position'] = point.positions[self.index_rotate_mobile_base]
                    self.goal_rotate_mobile_base['velocity'] = point.velocities[self.index_rotate_mobile_base] if len(point.velocities) > self.index_rotate_mobile_base else None
                    self.goal_rotate_mobile_base['acceleration'] = point.accelerations[self.index_rotate_mobile_base] if len(point.accelerations) > self.index_rotate_mobile_base else None
                    self.goal_rotate_mobile_base['contact_threshold'] = point.effort[self.index_rotate_mobile_base] if len(point.effort) > self.index_rotate_mobile_base else None

                if (self.goal_translate_mobile_base['position'] is not None) and \
                   (self.goal_rotate_mobile_base['position'] is not None):
                    err_str = ("Received a goal point with simultaneous translation and rotation mobile base goals. "
                               "This is not allowed. Only one is allowed to be sent for a given goal point. "
                               "translate_mobile_base = {0} and rotate_mobile_base = {1}").format(self.goal_translate_mobile_base['position'],
                                                                                                  self.goal_rotate_mobile_base['position'])
                    invalid_goal_callback(err_str)
                    return False
            else:
                goal_pos = point.positions[self.index] if len(point.positions) > self.index else None
                if goal_pos is None:
                    err_str = ("Received goal point with positions array length={0}. This joint ({1})'s index "
                               "is {2}. Length of array must cover all joints listed in "
                               "commanded_joint_names.").format(len(point.positions), self.name, self.index)
                    invalid_goal_callback(err_str)
                    return False

                self.goal['position'] = self.ros_to_mechaduino(goal_pos, kwargs['manipulation_origin'], fail_out_of_range_goal)
                self.goal['velocity'] = point.velocities[self.index] if len(point.velocities) > self.index else None
                self.goal['acceleration'] = point.accelerations[self.index] if len(point.accelerations) > self.index else None
                self.goal['contact_threshold'] = point.effort[self.index] if len(point.effort) > self.index else None
                if self.goal['position'] is None:
                    err_str = ("Received {0} goal point that is out of bounds. "
                               "Range = {1}, but goal point = {2}.").format(self.name, self.range, goal_pos)
                    invalid_goal_callback(err_str)
                    return False

        return True

    def ros_to_mechaduino(self, ros_ros, manipulation_origin, fail_out_of_range_goal):
        ros_pos = hm.bound_ros_command(self.range, ros_ros, fail_out_of_range_goal)
        return (manipulation_origin['x'] + ros_pos) if ros_pos is not None else None

    def init_execution(self, robot, robot_status, **kwargs):
        self.startx = robot_status['base']['x']
        self.starty = robot_status['base']['y']
        self.starttheta = robot_status['base']['theta']
        self.base_status = robot_status['base']

        if self.active:
            if self.active_translate_mobile_base or self.active_rotate_mobile_base:
                (_, mobile_base_error_m), (_, mobile_base_error_rad) = self.update_execution(robot_status)
                if mobile_base_error_m is not None:
                    robot.base.translate_by(mobile_base_error_m,
                                            v_m=self.goal_translate_mobile_base['velocity'],
                                            a_m=self.goal_translate_mobile_base['acceleration'],
                                            contact_thresh_N=self.goal_translate_mobile_base['contact_threshold'])
                elif mobile_base_error_rad is not None:
                    robot.base.rotate_by(mobile_base_error_rad,
                                         v_r=self.goal_rotate_mobile_base['velocity'],
                                         a_r=self.goal_rotate_mobile_base['acceleration'],
                                         contact_thresh_N=self.goal_rotate_mobile_base['contact_threshold'])
            else:
                robot.base.translate_by(self.update_execution(robot_status)[1],
                                        v_m=self.goal['velocity'],
                                        a_m=self.goal['acceleration'],
                                        contact_thresh_N=self.goal['contact_threshold'])

    def update_execution(self, robot_status, **kwargs):
        success_callback = kwargs['success_callback'] if 'success_callback' in kwargs.keys() else None
        currx = robot_status['base']['x']
        curry = robot_status['base']['y']
        currtheta = robot_status['base']['theta']
        self.base_status = robot_status['base']

        self.error = None
        self.error_translate_mobile_base_m = None
        self.error_rotate_mobile_base_rad = None
        if self.active:
            if self.active_translate_mobile_base or self.active_rotate_mobile_base:
                if self.goal_translate_mobile_base['position'] is not None:
                    if (robot_status['base']['left_wheel']['in_guarded_event'] or \
                        robot_status['base']['right_wheel']['in_guarded_event']) and \
                       success_callback:
                        success_callback("translate_mobile_base contact detected.")
                        return True
                    dist = np.sqrt(np.square(currx - self.startx) + np.square(curry - self.starty))
                    self.error_translate_mobile_base_m = self.goal_translate_mobile_base['position'] - (dist * np.sign(self.goal_translate_mobile_base['position']))
                    return [('translate_mobile_base', self.error_translate_mobile_base_m), ('rotate_mobile_base', self.error_rotate_mobile_base_rad)]
                elif self.goal_rotate_mobile_base['position'] is not None:
                    if (robot_status['base']['left_wheel']['in_guarded_event'] or \
                        robot_status['base']['right_wheel']['in_guarded_event']) and \
                       success_callback:
                        success_callback("rotate_mobile_base contact detected.")
                        return True
                    rot = hm.angle_diff_rad(currtheta, self.starttheta)
                    self.error_rotate_mobile_base_rad = hm.angle_diff_rad(self.goal_rotate_mobile_base['position'], rot)
                    return [('translate_mobile_base', self.error_translate_mobile_base_m), ('rotate_mobile_base', self.error_rotate_mobile_base_rad)]
            else:
                if (robot_status['base']['left_wheel']['in_guarded_event'] or \
                    robot_status['base']['right_wheel']['in_guarded_event']) and \
                   success_callback:
                    success_callback("{0} contact detected.".format(self.name))
                    return True
                self.error = self.goal['position'] - currx
                return self.name, self.error

        return None

    def goal_reached(self):
        if self.active:
            if self.active_translate_mobile_base or self.active_rotate_mobile_base:
                if self.active_translate_mobile_base:
                    reached = (abs(self.error_translate_mobile_base_m) < self.acceptable_mobile_base_error_m)
                    if not (abs(self.error_translate_mobile_base_m) < self.excellent_mobile_base_error_m):
                        # Use velocity to help decide when the low-level command has been finished
                        speed = np.sqrt(np.square(self.base_status['x_vel']) + np.square(self.base_status['y_vel']))
                        reached = reached and (speed < self.min_m_per_s)
                    return reached
                elif self.active_rotate_mobile_base:
                    reached = (abs(self.error_rotate_mobile_base_rad) < self.acceptable_mobile_base_error_rad)
                    if not (abs(self.error_rotate_mobile_base_rad) < self.excellent_mobile_base_error_rad):
                        # Use velocity to help decide when the low-level command has been finished
                        speed = self.base_status['theta_vel']
                        reached = reached and (abs(speed) < self.min_rad_per_s)
                    return reached
            else:
                return (abs(self.error) < self.acceptable_joint_error)

        return True
