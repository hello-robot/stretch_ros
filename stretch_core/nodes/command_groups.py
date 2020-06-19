#! /usr/bin/env python
from __future__ import print_function

import numpy as np
import hello_helpers.hello_misc as hm
from hello_helpers.gripper_conversion import GripperConversion


class SimpleCommandGroup:
    def __init__(self, joint_name, acceptable_joint_error=0.015, clip_ros_tolerance=0.001):
        self.clip_ros_tolerance = clip_ros_tolerance
        self.acceptable_joint_error = acceptable_joint_error
        self.joint_name = joint_name

    def update(self, joint_names, invalid_joints_error_func, **kwargs):
        self.use_joint = False
        if self.joint_name in joint_names:
            self.joint_index = joint_names.index(self.joint_name)
            self.use_joint = True
        return True

    def get_num_valid_commands(self):
        if self.use_joint:
            return 1
        return 0

    def set_goal(self, point, invalid_goal_error_func, **kwargs):
        self.joint_goal = False
        self.goal_joint_ros = None
        self.goal_joint_hello = None
        if self.use_joint:
            self.goal_joint_ros = point.positions[self.joint_index]
            self.goal_joint_hello = self.ros_to_mechaduino(self.goal_joint_ros)
            self.joint_goal = True
        if self.joint_goal and (self.goal_joint_hello is None):
            error_string = 'received goal point that is out of bounds. The first error that was caught is that the {0} goal is invalid ({1} = {2}).'.format(self.joint_name, self.joint_name, self.goal_joint_ros)
            invalid_goal_error_func(error_string)
            return False
        return True

    def ros_to_mechaduino(self, joint_ros):
        return joint_ros

    def init_execution(self, **kwargs):
        pass

    def update_execution(self, robot_status, **kwargs):
        # This method needs to be implemented. It also needs to set self.joint_error.
        return None

    def goal_reached(self):
        if self.joint_goal:
            return (abs(self.joint_error) < self.acceptable_joint_error)
        else:
            return True


class HeadPanCommandGroup(SimpleCommandGroup):
    def __init__(self, head_pan_calibrated_offset, head_pan_calibrated_looked_left_offset):
        SimpleCommandGroup.__init__(self, 'joint_head_pan', acceptable_joint_error=0.15, clip_ros_tolerance=0.001)
        self.head_pan_calibrated_offset = head_pan_calibrated_offset
        self.head_pan_calibrated_looked_left_offset = head_pan_calibrated_looked_left_offset

    def update_execution(self, robot_status, **kwargs):
        backlash_state = kwargs['backlash_state']
        if self.joint_goal:
            if backlash_state['head_pan_looked_left']:
                pan_backlash_correction = self.head_pan_calibrated_looked_left_offset
            else:
                pan_backlash_correction = 0.0
            self.current_joint_hello = robot_status['head']['head_pan']['pos'] + self.head_pan_calibrated_offset + pan_backlash_correction
            self.joint_error = self.goal_joint_hello - self.current_joint_hello
            self.joint_target = self.goal_joint_hello
            return self.joint_error
        else:
            return None


class HeadTiltCommandGroup(SimpleCommandGroup):
    def __init__(self, head_tilt_calibrated_offset, head_tilt_calibrated_looking_up_offset):
        SimpleCommandGroup.__init__(self, 'joint_head_tilt', acceptable_joint_error=0.52, clip_ros_tolerance=0.001)
        self.head_tilt_calibrated_offset = head_tilt_calibrated_offset
        self.head_tilt_calibrated_looking_up_offset = head_tilt_calibrated_looking_up_offset

    def update_execution(self, robot_status, **kwargs):
        backlash_state = kwargs['backlash_state']
        if self.joint_goal:
            if backlash_state['head_tilt_looking_up']:
                tilt_backlash_correction = self.head_tilt_calibrated_looking_up_offset
            else:
                tilt_backlash_correction = 0.0
            self.current_joint_hello = robot_status['head']['head_tilt']['pos'] + self.head_tilt_calibrated_offset + tilt_backlash_correction
            self.joint_error = self.goal_joint_hello - self.current_joint_hello
            self.joint_target = self.goal_joint_hello
            return self.joint_error
        else:
            return None


class WristYawCommandGroup(SimpleCommandGroup):
    def __init__(self):
        SimpleCommandGroup.__init__(self, 'joint_wrist_yaw', acceptable_joint_error=0.015, clip_ros_tolerance=0.001)

    def update_execution(self, robot_status, **kwargs):
        if self.joint_goal:
            self.current_joint_hello = robot_status['end_of_arm']['wrist_yaw']['pos']
            self.joint_error = self.goal_joint_hello - self.current_joint_hello
            self.joint_target = self.goal_joint_hello
            return self.joint_error
        else:
            return None


class GripperCommandGroup:
    def __init__(self, acceptable_joint_error=0.015, clip_ros_tolerance=0.001):
        self.clip_ros_tolerance = clip_ros_tolerance
        self.acceptable_joint_error = acceptable_joint_error
        self.gripper_joint_names = ['joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture']
        self.gripper_conversion = GripperConversion()

    def update(self, joint_names, invalid_joints_error_func, **kwargs):
        self.use_gripper_joint = False
        self.gripper_joints_to_command_names = [j for j in self.gripper_joint_names if j in joint_names]
        self.gripper_joints_to_command_indices = [joint_names.index(j) for j in self.gripper_joints_to_command_names]
        if len(self.gripper_joints_to_command_names) > 1:
            # Commands that attempt to control the gripper with more
            # than one joint name are not allowed, since the gripper
            # ultimately only has a single degree of freedom.
            error_string = 'Received a command for the gripper that includes more than one gripper joint name: {0}. Only one joint name is allowed to be used for a gripper command to avoid conflicts and confusion. The gripper only has a single degree of freedom that can be controlled using the following three mutually exclusive joint names: {1}.'.format(self.gripper_joints_to_command_names, self.gripper_joint_names)
            invalid_joints_error_func(error_string)
            self.use_gripper_joint = False
            return False

        if len(self.gripper_joints_to_command_names) == 1:
            self.gripper_joint_command_name = self.gripper_joints_to_command_names[0]
            self.gripper_joint_command_index = self.gripper_joints_to_command_indices[0]
            self.use_gripper_joint = True

        return True

    def get_num_valid_commands(self):
        if self.use_gripper_joint:
            return 1
        else:
            return 0

    def set_goal(self, point, invalid_goal_error_func, **kwargs):
        self.gripper_joint_goal = False
        self.goal_gripper_joint = None
        goal = None
        if self.use_gripper_joint:
            name = self.gripper_joint_command_name
            goal = point.positions[self.gripper_joint_command_index]
            if ((name == 'joint_gripper_finger_left') or (name == 'joint_gripper_finger_right')):
                self.goal_gripper_joint = self.gripper_conversion.finger_to_robotis(goal)
            if (name == 'gripper_aperture'):
                self.goal_gripper_joint = self.gripper_conversion.aperture_to_robotis(goal)
            self.gripper_joint_goal = True

            # Check that the goal is valid.
            self.gripper_joint_goal_valid = True
            if self.goal_gripper_joint is None:
                self.gripper_joint_goal_valid = False

            if not self.gripper_joint_goal_valid:
                error_string = 'gripper goal {0} is invalid'.format(goal)
                invalid_goal_error_func(error_string)
                return False
        return True

    def ros_to_mechaduino(self, joint_ros):
        return joint_ros

    def init_execution(self, **kwargs):
        pass

    def update_execution(self, robot_status, **kwargs):
        pass

    def goal_reached(self):
        # TODO: check the gripper state
        if self.use_gripper_joint:
            return True
        else:
            return True


class TelescopingCommandGroup:
    def __init__(self, wrist_extension_calibrated_retracted_offset):
        self.wrist_extension_calibrated_retracted_offset = wrist_extension_calibrated_retracted_offset
        self.telescoping_joints = ['joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0']
        self.clip_ros_tolerance = 0.001
        self.acceptable_joint_error_m = 0.005

    def update(self, joint_names, invalid_joints_error_func, **kwargs):
        self.use_telescoping_joints = False
        self.use_wrist_extension = False
        if 'wrist_extension' in joint_names:
            # Check if a wrist_extension command was received.
            self.use_wrist_extension = True
            self.extension_index = joint_names.index('wrist_extension')
            if any([(j in joint_names) for j in self.telescoping_joints]):
                # Consider commands for both the wrist_extension joint and any of the telescoping joints invalid, since these can be redundant.
                error_string = 'received a command for the wrist_extension joint and one or more telescoping_joints. These are mutually exclusive options. The joint names in the received command = {0}'.format(joint_names)
                invalid_joints_error_func(error_string)
                return False
        elif all([(j in joint_names) for j in self.telescoping_joints]):
            # Require all telescoping joints to be present for their commands to be used.
            self.use_telescoping_joints = True
            self.telescoping_indices = [joint_names.index(j) for j in self.telescoping_joints]
        return True

    def get_num_valid_commands(self):
        if self.use_wrist_extension:
            return 1
        if self.use_telescoping_joints:
            return len(self.telescoping_joints)
        return 0

    def set_goal(self, point, invalid_goal_error_func, **kwargs):
        self.extension_goal = False
        self.goal_extension_mecha = None
        self.goal_extension_ros = None
        if self.use_wrist_extension:
            self.goal_extension_ros = point.positions[self.extension_index]
            self.goal_extension_mecha = self.ros_to_mechaduino(self.goal_extension_ros)
            self.extension_goal = True

        if self.use_telescoping_joints:
            self.goal_extension_ros = sum([point.positions[i] for i in self.telescoping_indices])
            self.goal_extension_mecha = self.ros_to_mechaduino(self.goal_extension_ros)
            self.extension_goal = True

        if self.extension_goal and (self.goal_extension_mecha is None):
            error_string = 'received goal point that is out of bounds. The first error that was caught is that the extension goal is invalid (goal_extension_ros = {0}).'.format(self.goal_extension_ros)
            invalid_goal_error_func(error_string)
            return False
        return True

    def ros_to_mechaduino(self, wrist_extension_ros):
        return wrist_extension_ros

    def init_execution(self, **kwargs):
        pass

    def update_execution(self, robot_status, **kwargs):
        backlash_state = kwargs['backlash_state']
        if self.extension_goal:
            if backlash_state['wrist_extension_retracted']:
                arm_backlash_correction = self.wrist_extension_calibrated_retracted_offset
            else:
                arm_backlash_correction = 0.0
            self.current_extension_mecha = robot_status['arm']['pos'] + arm_backlash_correction
            self.extension_error_m = self.goal_extension_mecha - self.current_extension_mecha
            return self.extension_error_m
        else:
            return None

    def goal_reached(self):
        if self.extension_goal:
            return (abs(self.extension_error_m) < self.acceptable_joint_error_m)
        else:
            return True


class LiftCommandGroup:
    def __init__(self, max_arm_height):
        self.clip_ros_tolerance = 0.001
        self.acceptable_joint_error_m = 0.015 #15.0
        self.max_arm_height = max_arm_height
        self.lift_ros_range = [0.0, self.max_arm_height]

    def update(self, joint_names, invalid_joints_error_func, **kwargs):
        self.use_lift = False
        if 'joint_lift' in joint_names:
            self.lift_index = joint_names.index('joint_lift')
            self.use_lift = True
        return True

    def get_num_valid_commands(self):
        if self.use_lift:
            return 1
        return 0

    def set_goal(self, point, invalid_goal_error_func, **kwargs):
        self.lift_goal = False
        self.goal_lift_mecha = None
        self.goal_lift_ros = None
        if self.use_lift:
            self.goal_lift_ros = point.positions[self.lift_index]
            self.goal_lift_mecha = self.ros_to_mechaduino(self.goal_lift_ros)
            self.lift_goal = True

        if self.lift_goal and (self.goal_lift_mecha is None):
            error_string = 'received goal point that is out of bounds. The first error that was caught is that the lift goal is invalid (goal_lift_ros = {0}).'.format(self.goal_lift_ros)
            invalid_goal_error_func(error_string)
            return False
        return True

    def ros_to_mechaduino(self, lift_ros):
        return lift_ros

    def init_execution(self, **kwargs):
        pass

    def update_execution(self, robot_status, **kwargs):
        if self.lift_goal:
            self.current_lift_mecha = robot_status['lift']['pos']
            self.lift_error_m = self.goal_lift_mecha - self.current_lift_mecha
            return self.lift_error_m
        else:
            return None

    def goal_reached(self):
        if self.lift_goal:
            return (abs(self.lift_error_m) < self.acceptable_joint_error_m)
        else:
            return True


class MobileBaseCommandGroup:
    def __init__(self):
        self.mobile_base_virtual_joint_ros_range = [-0.5, 0.5]
        self.acceptable_mobile_base_error_m = 0.005
        self.excellent_mobile_base_error_m = 0.005
        self.acceptable_mobile_base_error_rad = (np.pi/180.0) * 6.0
        self.excellent_mobile_base_error_rad = (np.pi/180.0) * 0.6

    def update(self, joint_names, invalid_joints_error_func, **kwargs):
        robot_mode = kwargs['robot_mode']
        self.use_mobile_base_virtual_joint = False
        self.use_mobile_base_inc = False
        self.use_mobile_base_inc_rot = ('rotate_mobile_base' in joint_names)
        self.use_mobile_base_inc_trans = ('translate_mobile_base' in joint_names)
        self.use_mobile_base_inc = (self.use_mobile_base_inc_rot or self.use_mobile_base_inc_trans)
        self.use_mobile_base_virtual_joint = ('joint_mobile_base_translation' in joint_names)

        if self.use_mobile_base_inc and self.use_mobile_base_virtual_joint:
            # Commands that attempt to control the mobile base's
            # virtual joint together with mobile base incremental
            # commands are invalid.
            command_string = ''
            if self.use_mobile_base_inc_rot:
                command_string += ' rotate_mobile_base '
            if self.use_mobile_base_inc_trans:
                command_string += ' translate_mobile_base '
            error_string = 'received a command for the mobile base virtual joint (joint_mobile_base_translation) and mobile base incremental motions ({0}). These are mutually exclusive options. The joint names in the received command = {1}'.format(command_string, joint_names)
            invalid_joints_error_func(error_string)
            return False

        if self.use_mobile_base_virtual_joint:
            # Check if a mobile base command was received.
            if robot_mode != 'manipulation':
                error_string = 'must be in manipulation mode to receive a command for the joint_mobile_base_translation joint. Current mode = {0}.'.format(robot_mode)
                invalid_joints_error_func(error_string)
                return False
            self.virtual_joint_mobile_base_index = joint_names.index('joint_mobile_base_translation')

        if self.use_mobile_base_inc_rot:
            if robot_mode != 'position':
                error_string = 'must be in position mode to receive a rotate_mobile_base command. Current mode = {0}.'.format(robot_mode)
                invalid_joints_error_func(error_string)
                return False
            self.rotate_mobile_base_index = joint_names.index('rotate_mobile_base')

        if self.use_mobile_base_inc_trans:
            if robot_mode != 'position':
                error_string = 'must be in position mode to receive a translate_mobile_base command. Current mode = {0}.'.format(robot_mode)
                invalid_joints_error_func(error_string)
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

    def set_goal(self, point, invalid_goal_error_func, **kwargs):
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
            error_string = 'received a goal point with simultaneous rotation and translation mobile base goals. This is not allowed. Only one is allowed to be sent for a given goal point. rotate_mobile_base = {0} and translate_mobile_base = {1}'.format(self.goal_rotate_mobile_base_ros, self.goal_translate_mobile_base_ros)
            invalid_goal_error_func(error_string)
            return False

        return True

    def bound_ros_command(self, bounds, ros_pos, clip_ros_tolerance):
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
        ros_pos = self.bound_ros_command(bounds, ros_ros, self.clip_ros_tolerance)
        if ros_pos is None:
            return None
        else:
            return (manipulation_origin['x'] + ros_pos)

    def init_execution(self, **kwargs):
        robot_status = kwargs['robot_status']
        self.initial_mobile_base_translation_mecha_x = robot_status['base']['x']
        self.initial_mobile_base_translation_mecha_y = robot_status['base']['y']
        self.initial_mobile_base_rotation_mecha = robot_status['base']['theta']
        b = robot_status['base']

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
