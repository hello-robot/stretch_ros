#! /usr/bin/env python3

import numpy as np
import hello_helpers.hello_misc as hm
from hello_helpers.simple_command_group import SimpleCommandGroup
from hello_helpers.gripper_conversion import GripperConversion


class HeadPanCommandGroup(SimpleCommandGroup):
    def __init__(self, range_rad=None, calibrated_offset_rad=None, calibrated_looked_left_offset_rad=None, node=None):
        SimpleCommandGroup.__init__(self, 'joint_head_pan', range_rad, acceptable_joint_error=0.15, node=node)
        if calibrated_offset_rad is None:
            calibrated_offset_rad = node.controller_parameters['pan_angle_offset']
        self.calibrated_offset_rad = calibrated_offset_rad
        if calibrated_looked_left_offset_rad is None:
            calibrated_looked_left_offset_rad = node.controller_parameters['pan_looked_left_offset']
        self.looked_left_offset_rad = calibrated_looked_left_offset_rad
        self.looked_left = False

    def update_joint_range(self, joint_range, node=None):
        if joint_range is not None:
            self.range = joint_range
            return

        if node is None:
            return # cannot calculate range without Stretch Body handle
        range_ticks = node.robot.head.motors['head_pan'].params['range_t']
        range_rad = (node.robot.head.motors['head_pan'].ticks_to_world_rad(range_ticks[1]),
                     node.robot.head.motors['head_pan'].ticks_to_world_rad(range_ticks[0]))
        self.range = range_rad

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            _, pan_error = self.update_execution(robot_status)
            robot.head.move_by('head_pan', pan_error, v_r=self.goal['velocity'], a_r=self.goal['acceleration'])
            self.looked_left = pan_error > 0.0

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        if self.active:
            pan_backlash_correction = self.looked_left_offset_rad if self.looked_left else 0.0
            pan_current = robot_status['head']['head_pan']['pos'] + self.calibrated_offset_rad + pan_backlash_correction
            self.error = self.goal['position'] - pan_current
            return self.name, self.error

        return None

    def joint_state(self, robot_status, **kwargs):
        pan_status = robot_status['head']['head_pan']
        pan_backlash_correction = self.looked_left_offset_rad if self.looked_left else 0.0
        pos = pan_status['pos'] + self.calibrated_offset_rad + pan_backlash_correction
        return (pos, pan_status['vel'], pan_status['effort'])


class HeadTiltCommandGroup(SimpleCommandGroup):
    def __init__(self, range_rad=None, calibrated_offset_rad=None, calibrated_looking_up_offset_rad=None, backlash_transition_angle_rad=None, node=None):
        SimpleCommandGroup.__init__(self, 'joint_head_tilt', range_rad, acceptable_joint_error=0.52, node=node)
        if calibrated_offset_rad is None:
            calibrated_offset_rad = node.controller_parameters['tilt_angle_offset']
        self.calibrated_offset_rad = calibrated_offset_rad
        if calibrated_looking_up_offset_rad is None:
            calibrated_looking_up_offset_rad = node.controller_parameters['tilt_looking_up_offset']
        self.looking_up_offset_rad = calibrated_looking_up_offset_rad
        if backlash_transition_angle_rad is None:
            backlash_transition_angle_rad = node.controller_parameters['tilt_angle_backlash_transition']
        self.backlash_transition_angle_rad = backlash_transition_angle_rad
        self.looking_up = False

    def update_joint_range(self, joint_range, node=None):
        if joint_range is not None:
            self.range = joint_range
            return

        if node is None:
            return # cannot calculate range without Stretch Body handle
        range_ticks = node.robot.head.motors['head_tilt'].params['range_t']
        range_rad = (node.robot.head.motors['head_tilt'].ticks_to_world_rad(range_ticks[1]),
                     node.robot.head.motors['head_tilt'].ticks_to_world_rad(range_ticks[0]))
        self.range = range_rad

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            _, tilt_error = self.update_execution(robot_status)
            robot.head.move_by('head_tilt', tilt_error, v_r=self.goal['velocity'], a_r=self.goal['acceleration'])
            self.looking_up = self.goal['position'] > (self.backlash_transition_angle_rad + self.calibrated_offset_rad)

    def update_execution(self, robot_status, **kwargs):
        self.error = None
        if self.active:
            tilt_backlash_correction = self.looking_up_offset_rad if self.looking_up else 0.0
            tilt_current = robot_status['head']['head_tilt']['pos'] + self.calibrated_offset_rad + tilt_backlash_correction
            self.error = self.goal['position'] - tilt_current
            return self.name, self.error

        return None

    def joint_state(self, robot_status, **kwargs):
        tilt_status = robot_status['head']['head_tilt']
        tilt_backlash_correction = self.looking_up_offset_rad if self.looking_up else 0.0
        pos = tilt_status['pos'] + self.calibrated_offset_rad + tilt_backlash_correction
        return (pos, tilt_status['vel'], tilt_status['effort'])


class WristYawCommandGroup(SimpleCommandGroup):
    def __init__(self, range_rad=None, node=None):
        SimpleCommandGroup.__init__(self, 'joint_wrist_yaw', range_rad, node=node)

    def update_joint_range(self, joint_range, node=None):
        if joint_range is not None:
            self.range = joint_range
            return

        if node is None:
            return # cannot calculate range without Stretch Body handle
        range_ticks = node.robot.end_of_arm.motors['wrist_yaw'].params['range_t']
        range_rad = (node.robot.end_of_arm.motors['wrist_yaw'].ticks_to_world_rad(range_ticks[1]),
                     node.robot.end_of_arm.motors['wrist_yaw'].ticks_to_world_rad(range_ticks[0]))
        self.range = range_rad

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

    def joint_state(self, robot_status, **kwargs):
        yaw_status = robot_status['end_of_arm']['wrist_yaw']
        return (yaw_status['pos'], yaw_status['vel'], yaw_status['effort'])


class GripperCommandGroup(SimpleCommandGroup):
    def __init__(self, range_robotis=None, node=None):
        self.gripper_conversion = GripperConversion()
        SimpleCommandGroup.__init__(self, 'joint_gripper_finger_left', range_robotis, acceptable_joint_error=1.0, node=node)
        self.gripper_joint_names = ['joint_gripper_finger_left', 'joint_gripper_finger_right', 'gripper_aperture']
        self.update_joint_range(range_robotis)

    def update_joint_range(self, joint_range, node=None):
        if joint_range is not None:
            self.range = joint_range
            self.range_aperture_m = (self.gripper_conversion.robotis_to_aperture(joint_range[0]),
                                    self.gripper_conversion.robotis_to_aperture(joint_range[1]))
            self.range_finger_rad = (self.gripper_conversion.robotis_to_finger(joint_range[0]),
                                    self.gripper_conversion.robotis_to_finger(joint_range[1]))
            return

        if node is None:
            return # cannot calculate range without Stretch Body handle
        range_ticks = node.robot.end_of_arm.motors['stretch_gripper'].params['range_t']
        range_rad = (node.robot.end_of_arm.motors['stretch_gripper'].ticks_to_world_rad(range_ticks[0]),
                     node.robot.end_of_arm.motors['stretch_gripper'].ticks_to_world_rad(range_ticks[1]))
        range_robotis = (node.robot.end_of_arm.motors['stretch_gripper'].world_rad_to_pct(range_rad[0]),
                         node.robot.end_of_arm.motors['stretch_gripper'].world_rad_to_pct(range_rad[1]))
        self.range = range_robotis
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

    def joint_state(self, robot_status, **kwargs):
        joint_name = kwargs['joint_name'] if 'joint_name' in kwargs.keys() else self.name
        gripper_status = robot_status['end_of_arm']['stretch_gripper']
        pos_aperture_m, pos_rad, effort, vel = self.gripper_conversion.status_to_all(gripper_status)
        if (joint_name == 'gripper_aperture'):
            return (pos_aperture_m, vel, effort)
        elif (joint_name == 'joint_gripper_finger_left') or (joint_name == 'joint_gripper_finger_right'):
            return (pos_rad, vel, effort)


class ArmCommandGroup(SimpleCommandGroup):
    def __init__(self, range_m=None, calibrated_retracted_offset_m=None, node=None):
        SimpleCommandGroup.__init__(self, 'joint_arm', range_m, acceptable_joint_error=0.008, node=node)
        self.telescoping_joints = ['joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0']
        self.is_telescoping = False
        self.wrist_extension_name = 'wrist_extension'
        self.is_named_wrist_extension = False
        if calibrated_retracted_offset_m is None:
            calibrated_retracted_offset_m = node.controller_parameters['arm_retracted_offset']
        self.retracted_offset_m = calibrated_retracted_offset_m
        self.retracted = False

    def update_joint_range(self, joint_range, node=None):
        if joint_range is not None:
            self.range = joint_range
            return

        if node is None:
            return # cannot calculate range without Stretch Body handle
        range_m = tuple(node.robot.arm.params['range_m'])
        self.range = range_m

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
        if self.name in commanded_joint_names or self.wrist_extension_name in commanded_joint_names:
            if self.name in commanded_joint_names and self.wrist_extension_name in commanded_joint_names:
                err_str = ("Received a command for the joint_arm and wrist_extension joints. "
                           "These are mutually exclusive options. The joint names in the received command = "
                           "{0}").format(commanded_joint_names)
                invalid_joints_callback(err_str)
                return False
            if len(active_telescoping_joint_names) != 0:
                err_str = ("Received a command for the joint_arm/wrist_extension joint and one or more telescoping_joints. "
                           "These are mutually exclusive options. The joint names in the received command = "
                           "{0}").format(commanded_joint_names)
                invalid_joints_callback(err_str)
                return False
            if self.name in commanded_joint_names:
                self.index = commanded_joint_names.index(self.name)
                self.active = True
            else:
                self.index = commanded_joint_names.index(self.wrist_extension_name)
                self.is_named_wrist_extension = True
                self.active = True
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
                           "in commanded_joint_names.").format(len(point.positions), self.name if not self.is_named_wrist_extension else self.wrist_extension_name, self.index)
                invalid_goal_callback(err_str)
                return False

            self.goal['position'] = hm.bound_ros_command(self.range, goal_pos, fail_out_of_range_goal)
            if self.goal['position'] is None:
                err_str = ("Received {0} goal point that is out of bounds. "
                            "Range = {1}, but goal point = {2}.").format(self.name if not self.is_named_wrist_extension else self.wrist_extension_name, self.range, goal_pos)
                invalid_goal_callback(err_str)
                return False

        return True

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            _, extension_error_m = self.update_execution(robot_status, force_single=True)
            robot.arm.move_by(extension_error_m,
                              v_m=self.goal['velocity'],
                              a_m=self.goal['acceleration'],
                              contact_thresh_pos=self.goal['contact_threshold'],
                              contact_thresh_neg=-self.goal['contact_threshold'] \
                                                   if self.goal['contact_threshold'] is not None else None)
            self.retracted = extension_error_m < 0.0

    def update_execution(self, robot_status, **kwargs):
        success_callback = kwargs['success_callback'] if 'success_callback' in kwargs.keys() else None
        force_single = kwargs['force_single'] if 'force_single' in kwargs.keys() else False
        self.error = None
        if self.active:
            if success_callback and robot_status['arm']['motor']['in_guarded_event']:
                success_callback("{0} contact detected.".format(self.name if not self.is_named_wrist_extension else self.wrist_extension_name))
                return True
            arm_backlash_correction = self.retracted_offset_m if self.retracted else 0.0
            extension_current = robot_status['arm']['pos'] + arm_backlash_correction
            self.error = self.goal['position'] - extension_current
            if force_single:
                return self.name, self.error
            else:
                if self.is_telescoping:
                    return [(telescoping_joint, self.error / len(self.telescoping_joints)) for telescoping_joint in self.telescoping_joints]
                else:
                    return [(self.name, self.error), (self.wrist_extension_name, self.error)]

        return None

    def joint_state(self, robot_status, **kwargs):
        arm_status = robot_status['arm']
        arm_backlash_correction = self.retracted_offset_m if self.retracted else 0.0
        pos = arm_status['pos'] + arm_backlash_correction
        return (pos, arm_status['vel'], arm_status['motor']['effort_pct'])


class LiftCommandGroup(SimpleCommandGroup):
    def __init__(self, range_m=None, node=None):
        SimpleCommandGroup.__init__(self, 'joint_lift', range_m, node=node)

    def update_joint_range(self, joint_range, node=None):
        if joint_range is not None:
            self.range = joint_range
            return

        if node is None:
            return # cannot calculate range without Stretch Body handle
        range_m = tuple(node.robot.lift.params['range_m'])
        self.range = range_m

    def init_execution(self, robot, robot_status, **kwargs):
        if self.active:
            robot.lift.move_by(self.update_execution(robot_status)[1],
                               v_m=self.goal['velocity'],
                               a_m=self.goal['acceleration'],
                               contact_thresh_pos=self.goal['contact_threshold'],
                               contact_thresh_neg=-self.goal['contact_threshold'] \
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

    def joint_state(self, robot_status, **kwargs):
        lift_status = robot_status['lift']
        return (lift_status['pos'], lift_status['vel'], lift_status['motor']['effort_pct'])


class MobileBaseCommandGroup(SimpleCommandGroup):
    def __init__(self, virtual_range_m=(-0.5, 0.5), node=None):
        SimpleCommandGroup.__init__(self, 'joint_mobile_base_translation', virtual_range_m,
                                    acceptable_joint_error=0.005, node=node)
        self.incrementing_joint_names = ['translate_mobile_base', 'rotate_mobile_base']
        self.active_translate_mobile_base = False
        self.active_rotate_mobile_base = False
        self.acceptable_mobile_base_error_m = 0.005
        self.excellent_mobile_base_error_m = 0.005
        self.acceptable_mobile_base_error_rad = (np.pi/180.0) * 6.0
        self.excellent_mobile_base_error_rad = (np.pi/180.0) * 0.6
        self.min_m_per_s = 0.002
        self.min_rad_per_s = np.radians(1.0)

    def update_joint_range(self, virtual_joint_range, node=None):
        if virtual_joint_range is not None:
            self.range = virtual_joint_range
            return

        virtual_range_m = (-0.5, 0.5)
        self.range = virtual_range_m

    def get_num_valid_commands(self):
        if self.active:
            num_inc = self.active_translate_mobile_base + self.active_rotate_mobile_base
            return num_inc if num_inc > 0 else 1

        return 0

    def update(self, commanded_joint_names, invalid_joints_callback, **kwargs):
        #
        # desired behavior:
        #    + If both rotation and translation commands received, then return goal failure.
        #    + If only one received, check that the array length works with the index.
        #    + Can both commands be provided and one have a null goal of some sort? If so, should consider handling this case.

        robot_mode = kwargs['robot_mode']
        self.active = False
        self.active_translate_mobile_base = False
        self.active_rotate_mobile_base = False
        self.index = None
        self.index_translate_mobile_base = None
        self.index_rotate_mobile_base = None
        active_incrementing_joint_names = list(set(commanded_joint_names) & set(self.incrementing_joint_names))

        if robot_mode == 'manipulation':
            err_str = ("MobileBaseCommandGroup.update: Manipulation mode is no longer allowed.")
            invalid_joints_callback(err_str)
            return False
        
        if 'joint_mobile_base_translation' in commanded_joint_names:
            err_str = ("MobileBaseCommandGroup.update: Received a command for the mobile base virtual joint (joint_mobile_base_translation). This joint is no longer allowed.")
            invalid_joints_callback(err_str)
            return False

        if (('translate_mobile_base' in active_incrementing_joint_names) and
            ('rotate_mobile_base' in active_incrementing_joint_names)):
            err_str = ("MobileBaseCommandGroup.update: Both translate_mobile_base and rotate_mobile_base commands received. Only one can be given at a time. They are mutually exclusive.")
            invalid_joints_callback(err_str)
            return False
        
        if len(active_incrementing_joint_names) != 0:
            if robot_mode == 'position':
                self.active = True
                if 'translate_mobile_base' in active_incrementing_joint_names:
                    self.active_translate_mobile_base = True
                    self.index_translate_mobile_base = commanded_joint_names.index('translate_mobile_base')
                if 'rotate_mobile_base' in active_incrementing_joint_names:
                    self.active_rotate_mobile_base = True
                    self.index_rotate_mobile_base = commanded_joint_names.index('rotate_mobile_base')
            else:
                err_str = ("MobileBaseCommandGroup.update: Must be in position mode to receive a command for the {0} joint(s). "
                           "Current mode = {1}.").format(active_incrementing_joint_names, robot_mode)
                invalid_joints_callback(err_str)
                return False

        return True

    def set_goal(self, point, invalid_goal_callback, fail_out_of_range_goal, **kwargs):
        #
        # desired behavior:
        #    + If both rotation and translation commands received, then return goal failure.
        #    + If only one received, check that the array length works with the index.
        #    + Can both commands be provided and one have a null goal of some sort? If so, should consider handling this case.

        self.goal = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        self.goal_translate_mobile_base = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        self.goal_rotate_mobile_base = {"position": None, "velocity": None, "acceleration": None, "contact_threshold": None}
        if self.active:
            if self.active_translate_mobile_base and self.active_rotate_mobile_base:
                err_str = ("MobileBaseCommandGroup.set_goal: Both self.active_translate_mobile_base and self.active_rotate_mobile_base are True, which should not happen since only translation or rotation can be commanded at a time. They are mutually exclusive.")
                invalid_goal_callback(err_str)
                return False
                
            if self.active_translate_mobile_base:
                if (len(point.positions) <= self.index_translate_mobile_base):
                    err_str = ("MobileBaseCommandGroup.set_goal: Received goal point with positions array "
                               "length {0} and names ({1}), but the index for the mobile base translation "
                               "joint is {2}.").format(len(point.positions),
                                                       self.incrementing_joint_names,
                                                       self.index_translate_mobile_base)
                    invalid_goal_callback(err_str)
                    return False
                    
                if not np.isclose(point.positions[self.index_translate_mobile_base], 0.0, rtol=1e-5, atol=1e-8, equal_nan=False):
                    self.goal_translate_mobile_base['position'] = point.positions[self.index_translate_mobile_base]
                    self.goal_translate_mobile_base['velocity'] = point.velocities[self.index_translate_mobile_base] if len(point.velocities) > self.index_translate_mobile_base else None
                    self.goal_translate_mobile_base['acceleration'] = point.accelerations[self.index_translate_mobile_base] if len(point.accelerations) > self.index_translate_mobile_base else None
                    self.goal_translate_mobile_base['contact_threshold'] = point.effort[self.index_translate_mobile_base] if len(point.effort) > self.index_translate_mobile_base else None


            if self.active_rotate_mobile_base:
                if (len(point.positions) <= self.index_rotate_mobile_base):
                    err_str = ("MobileBaseCommandGroup.set_goal: Received goal point with positions array "
                               "length {0} and names ({1}), but the index for the mobile base rotation "
                               "joint is {2}.").format(len(point.positions),
                                                       self.incrementing_joint_names,
                                                       self.index_rotate_mobile_base)
                    invalid_goal_callback(err_str)
                    return False

                if not np.isclose(point.positions[self.index_rotate_mobile_base], 0.0, rtol=1e-5, atol=1e-8, equal_nan=False):
                    self.goal_rotate_mobile_base['position'] = point.positions[self.index_rotate_mobile_base]
                    self.goal_rotate_mobile_base['velocity'] = point.velocities[self.index_rotate_mobile_base] if len(point.velocities) > self.index_rotate_mobile_base else None
                    self.goal_rotate_mobile_base['acceleration'] = point.accelerations[self.index_rotate_mobile_base] if len(point.accelerations) > self.index_rotate_mobile_base else None
                    self.goal_rotate_mobile_base['contact_threshold'] = point.effort[self.index_rotate_mobile_base] if len(point.effort) > self.index_rotate_mobile_base else None
                                                                                  
        return True

                                                                                   
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
                                            contact_thresh=self.goal_translate_mobile_base['contact_threshold'])
                elif mobile_base_error_rad is not None:
                    robot.base.rotate_by(mobile_base_error_rad,
                                         v_r=self.goal_rotate_mobile_base['velocity'],
                                         a_r=self.goal_rotate_mobile_base['acceleration'],
                                         contact_thresh=self.goal_rotate_mobile_base['contact_threshold'])
            else:
                robot.base.translate_by(self.update_execution(robot_status)[1],
                                        v_m=self.goal['velocity'],
                                        a_m=self.goal['acceleration'],
                                        contact_thresh=self.goal['contact_threshold'])

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
