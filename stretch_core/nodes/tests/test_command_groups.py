import pytest
import math
import stretch_body.robot as robot

from .. import command_groups as cg

#Initalize fixtures for all possible command groups
@pytest.fixture
def head_pan_cg():
    range_rad = (-math.pi/4,math.pi/4)
    head_pan_calibrated_offset_rad = math.pi/16
    head_pan_calibrated_looked_left_offset_rad = 0.03
    return cg.HeadPanCommandGroup(range_rad, head_pan_calibrated_offset_rad,
                                   head_pan_calibrated_looked_left_offset_rad)

@pytest.fixture
def head_tilt_cg():
    range_rad = (-math.pi/4,math.pi/4)
    head_tilt_calibrated_offset_rad = math.pi/16
    head_tilt_calibrated_looking_up_offset_rad = 0.03
    head_tilt_backlash_transition_angle_rad = 0.2
    return cg.HeadTiltCommandGroup(range_rad,head_tilt_calibrated_offset_rad,
                                    head_tilt_calibrated_looking_up_offset_rad,
                                    head_tilt_backlash_transition_angle_rad)

@pytest.fixture
def wrist_yaw_cg():
    range_rad = (-math.pi/4,math.pi/4)
    return cg.WristYawCommandGroup(range_rad)

@pytest.fixture
def gripper_cg():
    range_robotis = (0,3)
    return cg.GripperCommandGroup(range_robotis)

@pytest.fixture
def telescoping_cg():
    range_m = (0,0.8)
    wrist_extension_calibrated_retracted_offset_m = 0.03
    return cg.TelescopingCommandGroup(range_m,
                                wrist_extension_calibrated_retracted_offset_m)

@pytest.fixture
def lift_cg():
    range_m = (0,1)
    return cg.LiftCommandGroup(range_m)

@pytest.fixture
def mobile_base_cg():
    virtual_range_m = (-0.5,0.5)
    return cg.MobileBaseCommandGroup(virtual_range_m)

@pytest.fixture
def invalid_joints_callback():
    pass


#Begin tests
def test_get_num_valid_commands(head_pan_cg,head_tilt_cg,wrist_yaw_cg,gripper_cg,telescoping_cg,lift_cg,mobile_base_cg):
    command_groups = [head_pan_cg,head_tilt_cg,wrist_yaw_cg,gripper_cg,
                      telescoping_cg,lift_cg,mobile_base_cg]

    failed_cg = [cg for cg in command_groups if cg.get_num_valid_commands() != 0]

    if len(failed_cg) == 0:
        assert True
    else:
        assert False
