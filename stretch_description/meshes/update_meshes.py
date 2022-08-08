#!/usr/bin/env python3
import os
from os.path import exists
import stretch_body.hello_utils as hu

if exists(hu.get_fleet_directory()+'stretch_configuration_params.yaml'):
    batch_name = hu.read_fleet_yaml('stretch_configuration_params.yaml')['robot']['batch_name'].lower()
else:
    batch_name = hu.read_fleet_yaml('stretch_re1_factory_params.yaml')['robot']['batch_name'].lower()
cmd='cp ~/catkin_ws/src/stretch_ros/stretch_description/meshes/'+batch_name+'/*.STL ~/catkin_ws/src/stretch_ros/stretch_description/meshes'
print("Copying in mesh files from batch:"+batch_name)
print(cmd)
os.system(cmd)
