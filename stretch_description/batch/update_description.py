#!/usr/bin/env python3
"""
This script updates the Mesh files and URDF file specific to a robot batch.
"""
import os
from os.path import exists
import stretch_body.hello_utils as hu
import subprocess
import sys


def run_cmd(cmdstr):
    process = subprocess.run(cmdstr, capture_output=True, text=True, shell=True)
    if process.returncode != 0:
        print("update_description.py ERROR: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    return process


if exists(hu.get_fleet_directory() + 'stretch_configuration_params.yaml'):
    batch_name = hu.read_fleet_yaml('stretch_configuration_params.yaml')['robot']['batch_name'].lower()
else:
    batch_name = hu.read_fleet_yaml('stretch_re1_factory_params.yaml')['robot']['batch_name'].lower()

batch_meshes_path = os.path.expanduser(
    '~/catkin_ws/src/stretch_ros/stretch_description/batch/' + batch_name) + '/meshes/*.STL'
batch_urdfs_path = os.path.expanduser('~/catkin_ws/src/stretch_ros/stretch_description/batch/' + batch_name) + '/urdf/*.xacro'

meshes_path = os.path.expanduser('~/catkin_ws/src/stretch_ros/stretch_description/meshes')
urdfs_path = os.path.expanduser('~/catkin_ws/src/stretch_ros/stretch_description/urdf')

cmd_cp_meshes = 'cp {} {}'.format(batch_meshes_path, meshes_path)
cmd_cp_urdfs = 'cp {} {}'.format(batch_urdfs_path, urdfs_path)

print("Copying in Mesh files and URDF files from batch:" + batch_name)
print(cmd_cp_meshes)
run_cmd(cmd_cp_meshes)

print(cmd_cp_urdfs)
run_cmd(cmd_cp_urdfs)
