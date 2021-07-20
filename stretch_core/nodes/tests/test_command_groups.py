import pytest 
import math 
from .. import command_groups as cg    


@pytest.fixture 
def head_pan_cg():
    range_rad = (0,100) 
    head_pan_calibrated_offset_rad = math.pi/16 
    head_pan_calibrated_looked_left_offset_rad = 0.03 
    return cg.HeadPanCommandGroup(range_rad, head_pan_calibrated_offset_rad, head_pan_calibrated_looked_left_offset_rad)


def test_get_num_valid_commands(head_pan_cg): 
    assert head_pan_cg.get_num_valid_commands() == 0 
