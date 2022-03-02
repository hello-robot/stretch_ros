#!/usr/bin/env python
import argparse
import os
import sys
import yaml
import numpy as np
from tabulate import tabulate
import math
from scipy.spatial.transform import Rotation
from subprocess import Popen, PIPE
import time

parser = argparse.ArgumentParser(description='Analyzes the recent Collected Test Rig Data')
parser.add_argument("--observation",help="Shows the computed error stats from the most recent collected data using the Testrig.", action="store_true")
parser.add_argument('-f', metavar='data_file', type=str, help='File path to specific sample data collected using Test Rig (Default:Most Recent Sample Data)')
args = parser.parse_args()


class TestRig_analyze:
    def __init__(self,data_filename=None):
        self.data_directory = os.path.expanduser('~/catkin_ws/src/stretch_ros/stretch_camera_testrig/data')
        self.nominal_poses_filename = os.path.expanduser('~/catkin_ws/src/stretch_ros/stretch_camera_testrig/config/testrig_marker_info.yaml')
        self.data_keys = ['base_left_marker_pose',
                          'base_right_marker_pose',
                          'wrist_inside_marker_pose',
                          'wrist_top_marker_pose',
                          'shoulder_marker_pose']

        self.realsense_fw = self.get_realsense_fw()

        self.data_dict = None
        self.euclidean_error_dict = None
        self.angle_rotation_error_dict = None

        self.data_filename = None
        self.data_capture_date = None
        self.number_samples = None

        if data_filename:
            self.data_filename = data_filename
        else:
            files = os.listdir(self.data_directory)
            files.sort()
            if files[-1].startswith("testrig_collected_data_"):
                self.data_filename = self.data_directory + '/' + files[-1]
        
        capture_date = self.data_filename.split('_')[-1]
        capture_date = capture_date.split('.')[0]
        self.data_capture_date = capture_date

    def get_realsense_fw(self):
        fw_details = Popen("rs-fw-update -l | grep -i 'firmware'", shell=True, bufsize=64, stdin=PIPE, stdout=PIPE, close_fds=True).stdout.read()
        fw_details = fw_details.split(',')[3]
        fw_version = fw_details.split(' ')[-1]
        return fw_version

    def testrg_data_parse(self,filename):
        # Parses the collected data in yaml
        # returns  disctionary  with list of 4x4 Transformation Matrix
        print('Test Rig Data File : {0}'.format(filename))
        data_dict = {}
        try:
            with open(filename, 'r') as file:
                data_list = yaml.safe_load(file)

            for key in self.data_keys:
                data_dict[key] = []

            N = len(data_list)
            print('Number of Samples found : {0}'.format(N))
            self.number_of_samples = N
            for observation in data_list:
                for key in self.data_keys:
                    marker_pose = observation['camera_measurements'][key]
                    if marker_pose!=None:
                        data_dict[key].append(np.array(marker_pose))
                    else:
                        data_dict[key].append(None)
            return data_dict
        except IOError:
            print('[Error]:Unable to open Testrig Data file: {0}'.format(filename))

    def get_euclidean_errors(self,data_dict,nominal_poses_dict):
        error_dict = {}
        Num_samples = len(data_dict[self.data_keys[0]])

        for key in self.data_keys:
            error_dict[key] = []
            if Num_samples!=len(data_dict[key]):
                print('[Warning]: Number of samples found inconsistent for each Marker tag.')

        for key in error_dict.keys():
            for i in range(Num_samples):
                if type(data_dict[key][i]) != type(None):
                    error_dict[key].append(self.euclidean_error(nominal_poses_dict[key],data_dict[key][i]))
                else:
                    error_dict[key].append(None)
        return error_dict

    def get_angle_rot_errors(self,data_dict,nominal_poses_dict):
        error_dict = {}
        Num_samples = len(data_dict[self.data_keys[0]])

        for key in self.data_keys:
            error_dict[key] = []
            if Num_samples!=len(data_dict[key]):
                print('[Warning]: Number of samples found inconsistent for each Marker tag.')

        for key in error_dict.keys():
            for i in range(Num_samples):
                if type(data_dict[key][i]) != type(None):
                    error_dict[key].append(self.angle_rot_error(nominal_poses_dict[key],data_dict[key][i]))
                else:
                    error_dict[key].append(None)
        return error_dict

    def angle_rot_error(self,r1,r2):
        rot1 = np.array(r1)[:3,:3]
        rot2 = np.array(r2)[:3,:3]
        rot12 = np.matmul(rot1.T,rot2)
        theta_error = np.arccos((np.trace(rot12)-1)/2)
        return float(theta_error)

    def euclidean_error(self,r1,r2):
        dist = np.linalg.norm(r1[:3,3] - r2[:3,3])
        return float(dist)

    def get_nominal_pose_dict(self,filename):
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)
            nominal_poses_dict = {}
            for key in data['testrig_aruco_marker_info'].keys():
                nominal_poses_dict[key] = np.array(data['testrig_aruco_marker_info'][key])
            return nominal_poses_dict
        except IOError:
            print('[Error]:Unable to open Testrig Nominal Poses file: {0}'.format(filename))

    def save_error_results(self):
        t = time.localtime()
        capture_date = self.data_capture_date
        testrig_results = []

        for i in range(self.number_of_samples):
            error = {'error_data':{}}
            for key in self.data_keys:
                vals = {}
                vals['angle_rotation_error'] = self.angle_rotation_error_dict[key][i]
                vals['euclidean_error'] = self.euclidean_error_dict[key][i]
                error['error_data'][key] = vals
            testrig_results.append(error)

        filename = self.data_directory + '/results/testrig_errors_data_' + capture_date + '.yaml'

        with open(filename, 'w') as file:
            documents = yaml.dump(testrig_results, file)
        print('Test Rig Computed Error Data Saved to : {}'.format(filename))

    def get_observations(self):
        self.data_dict = self.testrg_data_parse(self.data_filename)
        self.nominal_poses_dict = self.get_nominal_pose_dict(self.nominal_poses_filename)

        self.euclidean_error_dict = self.get_euclidean_errors(self.data_dict,self.nominal_poses_dict)
        self.angle_rotation_error_dict = self.get_angle_rot_errors(self.data_dict,self.nominal_poses_dict)

if args.observation:
    test_rig = TestRig_analyze()
    test_rig.get_observations()
    test_rig.save_error_results()

if args.f:
    data_file = args.f
