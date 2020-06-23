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

    def set_goal(self, point, invalid_goal_callback, **kwargs):
        """Sets goal for the joint group

        Sets and validates the goal point for the joints
        in this command group.

        Parameters
        ----------
        point: trajectory_msgs.JointTrajectoryPoint
            the target point for all joints
        invalid_goal_callback: func
            error callback for invalid goal

        Returns
        -------
        bool
            False if commanded goal invalid, else True
        """
        # TODO: validate commanded_joint_names and all arrays in JointTrajectoryPoint (positions/velocities/etc) are same size
        self.goal = {"position": None}
        if self.active:
            self.goal['position'] = point.positions[self.index]
            if self.goal['position'] is None:
                err_str = 'Received goal point that is out of bounds. The first error that was caught is that the {0} goal is invalid ({1} = {2}).'.format(self.name, self.name, self.goal['position'])
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
    def __init__(self, head_pan_calibrated_offset, head_pan_calibrated_looked_left_offset):
        SimpleCommandGroup.__init__(self, 'joint_head_pan', (0, 0), acceptable_joint_error=0.15)
        self.head_pan_calibrated_offset = head_pan_calibrated_offset
        self.head_pan_calibrated_looked_left_offset = head_pan_calibrated_looked_left_offset

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            pan_error = self.update_execution(robot_status, backlash_state=kwargs['backlash_state'])
            robot.head.move_by('head_pan', pan_error)
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
            return self.error

        return None


class HeadTiltCommandGroup(SimpleCommandGroup):
    def __init__(self, head_tilt_calibrated_offset,
                 head_tilt_calibrated_looking_up_offset,
                 head_tilt_backlash_transition_angle):
        SimpleCommandGroup.__init__(self, 'joint_head_tilt', (0, 0), acceptable_joint_error=0.52)
        self.head_tilt_calibrated_offset = head_tilt_calibrated_offset
        self.head_tilt_calibrated_looking_up_offset = head_tilt_calibrated_looking_up_offset
        self.head_tilt_backlash_transition_angle = head_tilt_backlash_transition_angle

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            tilt_error = self.update_execution(robot_status, backlash_state=kwargs['backlash_state'])
            robot.head.move_by('head_tilt', tilt_error)
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
            return self.error

        return None


class WristYawCommandGroup(SimpleCommandGroup):
    def __init__(self):
        SimpleCommandGroup.__init__(self, 'joint_wrist_yaw', (0, 0))

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            robot.end_of_arm.move_by('wrist_yaw', self.update_execution(robot_status))

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        if self.active:
            self.error = self.goal['position'] - robot_status['end_of_arm']['wrist_yaw']['pos']
            return self.error

        return None


class GripperCommandGroup(SimpleCommandGroup):
    def __init__(self):
        SimpleCommandGroup.__init__(self, None, (0, 0))
        self.gripper_joint_names = ['joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture']
        self.gripper_conversion = GripperConversion()
        self.gripper_joint_goal_valid = False

    def update(self, commanded_joint_names, invalid_joints_callback, **kwargs):
        self.active = False
        self.index = None
        active_gripper_joint_names = list(set(commanded_joint_names) & set(self.gripper_joint_names))
        if len(active_gripper_joint_names) > 1:
            err_str = 'Received a command for the gripper that includes more than one gripper joint name: {0}. \
                       Only one joint name is allowed to be used for a gripper command to avoid conflicts \
                       and confusion. The gripper only has a single degree of freedom that can be \
                       controlled using the following three mutually exclusive joint names: \
                       {1}.'.format(active_gripper_joint_names, self.gripper_joint_names)
            invalid_joints_callback(err_str)
            return False
        elif len(active_gripper_joint_names) == 1:
            self.name = active_gripper_joint_names[0]
            self.index = commanded_joint_names.index(self.name)
            self.active = True

        return True

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            gripper_error = self.update_execution(robot_status)
            if (self.name == 'gripper_aperture'):
                gripper_robotis_error = self.gripper_conversion.aperture_to_robotis(gripper_error)
            elif (self.name == 'joint_gripper_finger_left') or (self.name == 'joint_gripper_finger_right'):
                gripper_robotis_error = self.gripper_conversion.finger_to_robotis(gripper_error)
            robot.end_of_arm.move_by('stretch_gripper', gripper_robotis_error)

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        if self.active:
            robotis_pct = robot_status['end_of_arm']['stretch_gripper']['pos_pct']
            if (self.name == 'gripper_aperture'):
                gripper_current = self.gripper_conversion.robotis_to_aperture(robotis_pct)
            elif (self.name == 'joint_gripper_finger_left') or (self.name == 'joint_gripper_finger_right'):
                gripper_current = self.gripper_conversion.robotis_to_finger(robotis_pct)

            self.error = self.goal['position'] - gripper_current
            return self.error

        return None


class TelescopingCommandGroup(SimpleCommandGroup):
    def __init__(self, wrist_extension_calibrated_retracted_offset):
        SimpleCommandGroup.__init__(self, 'wrist_extension', (0, 0), acceptable_joint_error=0.005)
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
                err_str = 'Received a command for the wrist_extension joint and one or more telescoping_joints. \
                           These are mutually exclusive options. The joint names in the received command = \
                           {0}'.format(commanded_joint_names)
                invalid_joints_callback(err_str)
                return False
        elif len(active_telescoping_joint_names) != 0:
            if len(active_telescoping_joint_names) == len(self.telescoping_joints):
                self.active = True
                self.is_telescoping = True
                self.index = [commanded_joint_names.index(i) for i in self.telescoping_joints]
            else:
                err_str = 'Commands with telescoping joints requires all telescoping joints to be present. \
                           Only received {0} of {1} telescoping joints. They are = \
                           {2}'.format(len(active_telescoping_joint_names), len(self.telescoping_joints),
                                       active_telescoping_joint_names)
                invalid_joints_callback(err_str)
                return False

        return True

    def set_goal(self, point, invalid_goal_callback, **kwargs):
        self.goal = {"position": None}
        if self.active:
            self.goal['position'] = sum([point.positions[i] for i in self.index]) \
                                    if self.is_telescoping else point.positions[self.index]
            if self.goal['position'] is None:
                err_str = 'Received extension goal point that is out of bounds. \
                           Goal point = {0}.'.format(self.goal['position'])
                invalid_goal_callback(err_str)
                return False

        return True

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            extension_error_m = self.update_execution(robot_status, backlash_state=kwargs['backlash_state'])
            robot.arm.move_by(extension_error_m)
            if extension_error_m < 0.0:
                kwargs['backlash_state']['wrist_extension_retracted'] = True
            else:
                kwargs['backlash_state']['wrist_extension_retracted'] = False

    def update_execution(self, robot_status, **kwargs):
        backlash_state = kwargs['backlash_state']
        self.error = None
        if self.active:
            if backlash_state['wrist_extension_retracted']:
                arm_backlash_correction = self.wrist_extension_calibrated_retracted_offset
            else:
                arm_backlash_correction = 0.0
            extension_current = robot_status['arm']['pos'] + arm_backlash_correction
            self.error = self.goal['position'] - extension_current
            return self.error

        return None


class LiftCommandGroup(SimpleCommandGroup):
    def __init__(self, max_arm_height):
        SimpleCommandGroup.__init__(self, 'joint_lift', (0.0, max_arm_height))

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            robot.lift.move_by(self.update_execution(robot_status))

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        if self.active:
            self.error = self.goal['position'] - robot_status['lift']['pos']
            return self.error

        return None


class MobileBaseCommandGroup:
    def __init__(self):
        self.mobile_base_virtual_joint_ros_range = [-0.5, 0.5]
        self.acceptable_mobile_base_error_m = 0.005
        self.excellent_mobile_base_error_m = 0.005
        self.acceptable_mobile_base_error_rad = (np.pi/180.0) * 6.0
        self.excellent_mobile_base_error_rad = (np.pi/180.0) * 0.6

    def update(self, joint_names, invalid_joints_callback, **kwargs):
        robot_mode = kwargs['robot_mode']
        self.use_mobile_base_inc_rot = ('rotate_mobile_base' in joint_names)
        self.use_mobile_base_inc_trans = ('translate_mobile_base' in joint_names)
        self.use_mobile_base_virtual_joint = ('joint_mobile_base_translation' in joint_names)

        if (self.use_mobile_base_inc_trans or self.use_mobile_base_inc_rot) \
            and self.use_mobile_base_virtual_joint:
            # Commands that attempt to control the mobile base's
            # virtual joint together with mobile base incremental
            # commands are invalid.
            command_string = ''
            if self.use_mobile_base_inc_rot:
                command_string += ' rotate_mobile_base '
            if self.use_mobile_base_inc_trans:
                command_string += ' translate_mobile_base '
            err_str = 'Received a command for the mobile base virtual joint (joint_mobile_base_translation) \
                       and mobile base incremental motions ({0}). These are mutually exclusive options. \
                       The joint names in the received command = {1}'.format(command_string, joint_names)
            invalid_joints_callback(err_str)
            return False

        if self.use_mobile_base_virtual_joint:
            if robot_mode != 'manipulation':
                err_str = 'Must be in manipulation mode to receive a command for the \
                           joint_mobile_base_translation joint. Current mode = {0}.'.format(robot_mode)
                invalid_joints_callback(err_str)
                return False
            self.virtual_joint_mobile_base_index = joint_names.index('joint_mobile_base_translation')

        if self.use_mobile_base_inc_rot:
            if robot_mode != 'position':
                err_str = 'Must be in position mode to receive a rotate_mobile_base command. \
                           Current mode = {0}.'.format(robot_mode)
                invalid_joints_callback(err_str)
                return False
            self.rotate_mobile_base_index = joint_names.index('rotate_mobile_base')

        if self.use_mobile_base_inc_trans:
            if robot_mode != 'position':
                err_str = 'Must be in position mode to receive a translate_mobile_base command. \
                           Current mode = {0}.'.format(robot_mode)
                invalid_joints_callback(err_str)
                return False
            self.translate_mobile_base_index = joint_names.index('translate_mobile_base')

        return True

    def get_num_valid_commands(self):
        number_of_valid_joints = 0
        if self.use_mobile_base_virtual_joint:
            number_of_valid_joints += 1
        if self.use_mobile_base_inc_rot:
            number_of_valid_joints += 1
        if self.use_mobile_base_inc_trans:
            number_of_valid_joints += 1
        return number_of_valid_joints

    def set_goal(self, point, invalid_goal_callback, **kwargs):
        self.rotate_mobile_base_goal = False
        self.translate_mobile_base_goal = False
        self.virtual_joint_mobile_base_goal = False

        if self.use_mobile_base_virtual_joint:
            self.goal_mobile_base_virtual_joint_ros = point.positions[self.virtual_joint_mobile_base_index]
            self.goal_mobile_base_virtual_joint_mecha = self.ros_to_mechaduino(self.goal_mobile_base_virtual_joint_ros, kwargs['manipulation_origin'])
            self.virtual_joint_mobile_base_goal = True

        if self.use_mobile_base_inc_rot:
            self.goal_rotate_mobile_base_mecha = point.positions[self.rotate_mobile_base_index]
            self.rotate_mobile_base_goal = True

        if self.use_mobile_base_inc_trans:
            self.goal_translate_mobile_base_mecha = point.positions[self.translate_mobile_base_index]
            self.translate_mobile_base_goal = True

        if self.rotate_mobile_base_goal and self.translate_mobile_base_goal:
            err_str = 'Received a goal point with simultaneous rotation and translation mobile base goals. \
                       This is not allowed. Only one is allowed to be sent for a given goal point. \
                       rotate_mobile_base = {0} and translate_mobile_base = \
                       {1}'.format(self.goal_rotate_mobile_base_mecha, self.goal_translate_mobile_base_mecha)
            invalid_goal_callback(err_str)
            return False

        return True

    def bound_ros_command(self, bounds, ros_pos, clip_ros_tolerance=0.001):
        # Clip the command with clip_ros_tolerance, instead of
        # invalidating it, if it is close enough to the valid ranges.
        if ros_pos < bounds[0]:
            # Command is lower than the minimum value.
            if (bounds[0] - ros_pos) < clip_ros_tolerance:
                return bounds[0]
            else:
                return None
        if ros_pos > bounds[1]:
            # Command is greater than the maximum value.
            if (ros_pos - bounds[1]) < clip_ros_tolerance:
                return bounds[1]
            else:
                return None
        return ros_pos

    def ros_to_mechaduino(self, ros_ros, manipulation_origin):
        bounds = self.mobile_base_virtual_joint_ros_range
        ros_pos = self.bound_ros_command(bounds, ros_ros)
        if ros_pos is None:
            return None
        else:
            return (manipulation_origin['x'] + ros_pos)

    def init_execution(self, robot, robot_status, **kwargs):
        self.initial_mobile_base_translation_mecha_x = robot_status['base']['x']
        self.initial_mobile_base_translation_mecha_y = robot_status['base']['y']
        self.initial_mobile_base_rotation_mecha = robot_status['base']['theta']
        b = robot_status['base']

        mobile_base_error_m, mobile_base_error_rad = self.update_execution(robot_status)
        if mobile_base_error_m is not None:
            robot.base.translate_by(mobile_base_error_m)
        if mobile_base_error_rad is not None:
            robot.base.rotate_by(mobile_base_error_rad)

    def update_execution(self, robot_status, **kwargs):
        current_mobile_base_translation_mecha_x = robot_status['base']['x']
        current_mobile_base_translation_mecha_y = robot_status['base']['y']
        current_mobile_base_rotation_mecha = robot_status['base']['theta']
        b = robot_status['base']

        self.mobile_base_error_m = None
        self.mobile_base_error_rad = None
        self.mobile_base_goal_reached = True
        if self.virtual_joint_mobile_base_goal:
            self.mobile_base_error_m = self.goal_mobile_base_virtual_joint_mecha - current_mobile_base_translation_mecha_x
            self.mobile_base_goal_reached = (abs(self.mobile_base_error_m) < self.acceptable_mobile_base_error_m)
        elif self.translate_mobile_base_goal:
            # incremental motion relative to the initial position
            x0 = self.initial_mobile_base_translation_mecha_x
            y0 = self.initial_mobile_base_translation_mecha_y
            x1 = current_mobile_base_translation_mecha_x
            y1 = current_mobile_base_translation_mecha_y
            distance_traveled = np.sqrt(np.square(x1 - x0) + np.square(y1 - y0))
            self.mobile_base_error_m = abs(self.goal_translate_mobile_base_mecha) - distance_traveled
            self.mobile_base_error_m = np.sign(self.goal_translate_mobile_base_mecha) * self.mobile_base_error_m
            self.mobile_base_goal_reached = (abs(self.mobile_base_error_m) < self.acceptable_mobile_base_error_m)
            if (abs(self.mobile_base_error_m) < self.excellent_mobile_base_error_m):
                self.mobile_base_goal_reached = True
            else:
                # Use velocity to help decide when the low-level command
                # has been finished.
                min_m_per_s = 0.002
                b = robot_status['base']
                speed = np.sqrt(np.square(b['x_vel']) + np.square(b['y_vel']))
                self.mobile_base_goal_reached = self.mobile_base_goal_reached and (speed < min_m_per_s)

        elif self.rotate_mobile_base_goal:
            incremental_rad = hm.angle_diff_rad(current_mobile_base_rotation_mecha, self.initial_mobile_base_rotation_mecha)
            self.mobile_base_error_rad = hm.angle_diff_rad(self.goal_rotate_mobile_base_mecha, incremental_rad)
            self.mobile_base_goal_reached = (abs(self.mobile_base_error_rad) < self.acceptable_mobile_base_error_rad)
            if (abs(self.mobile_base_error_rad) < self.excellent_mobile_base_error_rad):
                self.mobile_base_goal_reached = True
            else:
                # Use velocity to help decide when the low-level command
                # has been finished.
                min_deg_per_s = 1.0
                min_rad_per_s = min_deg_per_s * (np.pi/180.0)
                self.mobile_base_goal_reached = self.mobile_base_goal_reached and (abs(robot_status['base']['theta_vel']) < min_rad_per_s)
        return self.mobile_base_error_m, self.mobile_base_error_rad

    def goal_reached(self):
        return self.mobile_base_goal_reached
