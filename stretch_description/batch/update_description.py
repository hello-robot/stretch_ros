#!/usr/bin/env python3
import os
from os.path import exists
import stretch_body.hello_utils as hu

if exists(hu.get_fleet_directory() + 'stretch_configuration_params.yaml'):
    batch_name = hu.read_fleet_yaml('stretch_configuration_params.yaml')['robot']['batch_name'].lower()
else:
    batch_name = hu.read_fleet_yaml('stretch_re1_factory_params.yaml')['robot']['batch_name'].lower()

cmd_cp_meshes = 'cp ~/catkin_ws/src/stretch_ros/stretch_description/batch/' + batch_name + '/meshes/*.STL ~/catkin_ws/src/stretch_ros/stretch_description/meshes'
cmd_cp_urdfs = 'cp ~/catkin_ws/src/stretch_ros/stretch_description/batch/' + batch_name + '/urdf/* ~/catkin_ws/src/stretch_ros/stretch_description/meshes'

print("Copying in Mesh files and URDF files from batch:" + batch_name)
print(cmd_cp_meshes)
os.system(cmd_cp_meshes)
print(cmd_cp_urdfs)
os.system(cmd_cp_urdfs)
