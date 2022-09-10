#!/usr/bin/env python
from __future__ import print_function
import os
from os.path import exists
import stretch_body.hello_utils as hu
import sys

if exists(hu.get_fleet_directory() + 'stretch_configuration_params.yaml'):
    batch_name = hu.read_fleet_yaml('stretch_configuration_params.yaml')['robot']['batch_name'].lower()
else:
    batch_name = hu.read_fleet_yaml('stretch_re1_factory_params.yaml')['robot']['batch_name'].lower()

cmd='cp ~/catkin_ws/src/stretch_ros/stretch_description/meshes/'+batch_name+'/*.STL ~/catkin_ws/src/stretch_ros/stretch_description/meshes'
print("Copying in mesh files from batch: "+batch_name)
print(cmd)
if os.system(cmd) != 0:
    print('update_meshes.py ERROR: failed to execute copying command', file=sys.stderr)
    sys.exit(1)
