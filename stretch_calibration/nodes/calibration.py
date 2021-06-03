#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point

import math
import time
import threading
import sys

import numpy as np

from copy import deepcopy

import yaml
import time
import glob
import argparse as ap

from urdf_parser_py.urdf import URDF as urdf_parser
import urdf_parser_py as up

from scipy.spatial.transform import Rotation
import cma

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from hello_helpers.hello_misc import *
import hello_helpers.hello_ros_viz as hr


def use_all_data(sample_number, sample):
    # use all samples
    return True

def use_very_little_data(sample_number, sample):
    # use every twentieth sample
    if (sample_number % 20) == 0:
        return True
    else:
        return False

def use_all_aruco_and_some_accel(sample_number, sample):
    # Use all samples with one or more ArUco marker observations, and
    # a seventh of the remaining samples.
    c = sample['camera_measurements']

    wrist_inside_marker_detected = (c['wrist_inside_marker_pose'] != None)
    wrist_top_marker_detected = (c['wrist_top_marker_pose'] != None)
    base_left_marker_detected = (c['base_left_marker_pose'] != None)
    base_right_marker_detected = (c['base_right_marker_pose'] != None)

    # If an ArUco marker is detected, use the sample
    if wrist_top_marker_detected or wrist_inside_marker_detected or base_left_marker_detected or base_right_marker_detected:
        return True
    # For all other samples (accelerometer samples) use only some of the samples.
    if (sample_number % 7) == 0:
        return True
    return False

def soft_constraint_error(x, thresh, scale):
    # Returns a soft constraint error that is zero below the
    # constraint and increases linearly after the constraint.
    if x > thresh:
        return scale * (x - thresh)
    else:
        return 0.0
    
def affine_matrix_difference(t1, t2, size=4):
    error = 0.0
    for i in range(size):
        for j in range(size):
            error += abs(t1[i,j] - t2[i,j])
    return error

def rot_to_axes(r): 
    x_axis = np.reshape(r[:3,0], (3,1))
    y_axis = np.reshape(r[:3,1], (3,1))
    z_axis = np.reshape(r[:3,2], (3,1))
    return [x_axis, y_axis, z_axis]

def norm_axes(axes):
    x_axis, y_axis, z_axis = axes
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    return [x_axis, y_axis, z_axis]

def quat_to_rotated_axes(rot_mat, q):
    #r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_dcm()
    r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    rotated_r = np.matmul(rot_mat, r)
    return rot_to_axes(rotated_r)

def axis_error(axis, axis_target):
    # dot product comparison: 1.0 is best case and -1.0 is worst case
    error = np.dot(axis_target.transpose(), axis)
    # linear transform: 0.0 is best case and 1.0 is worst case
    return (1.0 - error) / 2.0

def axes_error(axes, axes_target):
    # 0.0 is the best case and 1.0 is the worst case
    errors = np.array([axis_error(axis, axis_target) for axis, axis_target in zip(axes, axes_target)])
    return np.sum(errors)/3.0

class Joint:
    # wrapper for a URDF joint
    
    def __init__(self, urdf_joint):
        self.joint = urdf_joint
        self.is_joint = True
        self.is_link = False
        
    def get_affine_matrix(self, value):
        axis = self.joint.axis
        rpy = self.joint.origin.rpy
        xyz = self.joint.origin.xyz
        affine_matrix = np.identity(4)
        affine_matrix[:3, 3] = xyz
        #affine_matrix[:3,:3] = Rotation.from_euler('xyz', rpy).as_dcm()
        affine_matrix[:3,:3] = Rotation.from_euler('xyz', rpy).as_matrix()
        if self.joint.type == 'revolute':
            axis_affine_matrix = np.identity(4)
            rotation_vector = value * (np.array(axis)/np.linalg.norm(axis))
            #axis_affine_matrix[:3,:3] = Rotation.from_rotvec(rotation_vector).as_dcm()
            axis_affine_matrix[:3,:3] = Rotation.from_rotvec(rotation_vector).as_matrix()
            affine_matrix = np.matmul(affine_matrix, axis_affine_matrix)
        elif self.joint.type == 'prismatic':
            axis_affine_matrix = np.identity(4)
            axis_affine_matrix[:3, 3] = value * (np.array(axis)/np.linalg.norm(axis))
            affine_matrix = np.matmul(affine_matrix, axis_affine_matrix)
        elif self.joint.type == 'fixed':
            affine_matrix = affine_matrix
        else:
            print('ERROR: unrecognized joint type = ', self.joint_type)
            exit()
        return affine_matrix

    def __str__(self):
        return self.joint.__str__()

class Link:
    # wrapper for a URDF link
    
    def __init__(self, urdf_link):
        self.link = urdf_link
        self.is_joint = False
        self.is_link = True

    def __str__(self):
        return self.link.__str__()
    
class Chain:
    # wrapper for a URDF chain
    
    def __init__(self, urdf, start_name, end_name):
        self.urdf = urdf
        self.chain_names = self.urdf.get_chain(start_name, end_name)
        self.chain = []
        for name in self.chain_names:
            is_joint = name in self.urdf.joint_map
            is_link = name in self.urdf.link_map
            if is_joint and is_link:
                print('ERROR: a joint and a link have the same name. This is not supported.')
                exit()
            if is_joint:
                self.chain.append(Joint(self.urdf.joint_map[name]))
            elif is_link:
                self.chain.append(Link(self.urdf.link_map[name]))
            else:
                print('ERROR: no joint or link was found with the name returned by get_chain')
                exit()

    def get_joint_by_name(self, name):
        for e in self.chain:
            if e.is_joint:
                if e.joint.name == name:
                    return e.joint
        return None
                
    def get_affine_matrix(self, joint_value_dict):
        affine_matrix = np.identity(4)
        for e in self.chain:
            if e.is_joint:
                value = joint_value_dict.get(e.joint.name, 0.0)
                affine_matrix = np.matmul(affine_matrix, e.get_affine_matrix(value))
        return affine_matrix


class ArucoError:
    # This object handles error calculations for a single ArUco
    # marker. For an example of its use, please see
    # process_head_calibration_data.
    
    def __init__(self, name, location, aruco_link, urdf, meters_per_deg, rgba):

        self.aruco_link = aruco_link

        # This creates a wrapper around a mutable URDF object that is
        # shared across multiple chains. When calibrating the URDF,
        # the URDF and this chain are updated outside of this
        # ArucoError object.
        #self.marker_frame = '/base_link'
        self.marker_frame = 'base_link'
        self.aruco_chain = Chain(urdf, 'base_link', self.aruco_link)
        
        self.rgba = rgba
        self.name = name
        self.location = location
        self.detected = False
        self.observed_aruco_pose = None
        self.joint_values = None
        self.meters_per_deg = meters_per_deg
        self.number_of_observations = None

    def reset_observation_count(self):
        # Set observation count to zero.
        self.number_of_observations = 0

    def increment_observation_count(self, sample):
        # Increments the observation count if the provided sample
        # includes an observation of this ArUco marker.
        c = sample['camera_measurements']
        if (c[self.name + '_marker_pose'] != None):
            self.number_of_observations += 1
          
    def update(self, sample, marker_time, unused):
        # Use the sample to update the ArUco marker's observed pose
        # and the corresponding robot joint configuration.
        camera_measurements = sample['camera_measurements']
        self.detected = (camera_measurements.get(self.name + '_marker_pose') != None)
        if self.detected is None:
            self.detected = False
            self.observed_aruco_pose = None
        else: 
            self.observed_aruco_pose = camera_measurements[self.name + '_marker_pose']
        self.joint_values = sample['joints']
        self.marker_time = marker_time

    def get_target_ros_markers(self, sample, marker_id, marker_time, rgba, unused):
        # If this ArUco marker was observed in the sample, create ROS
        # markers for visualization of this ArUco marker as predicted
        # by the current URDF using the robot's configuration for the
        # sample.
        ros_markers = []
        
        camera_measurements = sample['camera_measurements']
        detected = (camera_measurements.get(self.name + '_marker_pose') != None)

        if (detected is None) or (not detected):
            return [], marker_id

        joint_values = sample['joints']
        marker_time = marker_time

        target_transform = self.aruco_chain.get_affine_matrix(joint_values)
        target_xyz = target_transform[:3,3]
        target_axes = rot_to_axes(target_transform[:3,:3])

        xyz = target_xyz
        marker = hr.create_sphere_marker(xyz, marker_id, self.marker_frame, marker_time, rgba)
        marker_id += 1
        ros_markers.append(marker)

        z_axis = target_axes[2]
        marker = hr.create_axis_marker(xyz, z_axis, marker_id, self.marker_frame, marker_time, rgba)
        marker_id += 1
        ros_markers.append(marker)

        # A list of ROS markers for visualization, and an ID number
        # for the last ROS marker generated.
        return ros_markers, marker_id

    
    def error(self, camera_transform, fit_z, fit_orientation, marker_id, visualize=True, visualize_targets=False):
        # Calculate the error between the observed ArUco marker pose
        # and the ArUco marker pose predicted by the current URDF (the
        # target pose). The update method must be called before
        # calling this method. This method should not be called if the
        # ArUco marker was not observed. For example, if self.detected
        # is False, then self.observed_aruco_pose will be None and
        # this method will fail.

        # Initialize the position and orientation error. If
        # fit_orientation is False, the orientation error will not be
        # updated.
        error = {'pos': 0.0,
                 'orient': 0.0}

        # Find the ArUco marker pose predicted by the current URDF
        # (the target pose).
        target_transform = self.aruco_chain.get_affine_matrix(self.joint_values)
        target_xyz = target_transform[:3,3]
        target_axes = rot_to_axes(target_transform[:3,:3])

        # Transform the camera observation of the ArUco marker into
        # the world coordinate system using the provided camera
        # transform.
        p = self.observed_aruco_pose.position
        observed_xyz = np.dot(camera_transform, np.array([p.x, p.y, p.z, 1.0]))[:3]
        if fit_orientation: 
            q = self.observed_aruco_pose.orientation
            observed_axes = quat_to_rotated_axes(camera_transform[:3,:3], q)

        # With respect to the world frame, calculate errors by
        # comparing the transformed camera observation of the ArUco
        # marker with the current URDF's ArUco marker prediction.
        if not fit_z:
            # Only fit planar direction to the marker
            # position due to downward deflection of the
            # telescoping arm.
            vec1 = observed_xyz[:2]
            vec2 = target_xyz[:2]
            vec1 = vec1 / np.linalg.norm(vec1)
            vec2 = vec2 / np.linalg.norm(vec2)
            error['pos'] += (self.meters_per_deg * axis_error(vec1, vec2)) / self.number_of_observations
        else:
            if fit_orientation:
                # Calculate the position and orientation errors.
                error['pos'] += np.linalg.norm(observed_xyz - target_xyz) / self.number_of_observations
                # 0.0 is the best case and 1.0 is the worst case
                error['orient'] += (self.meters_per_deg * axes_error(observed_axes, target_axes)) / self.number_of_observations
            else:
                # Only calculate the position error.
                error['pos'] += np.linalg.norm(observed_xyz - target_xyz) / self.number_of_observations

        # If requested, generate ROS markers to visualize the pose of
        # the ArUco observation and/or the pose of the ArUco marker
        # predicted by the current URDF.
        ros_markers = []
        if visualize:
            xyz = observed_xyz
            marker = hr.create_sphere_marker(xyz, marker_id, self.marker_frame, self.marker_time, self.rgba)
            marker_id += 1
            ros_markers.append(marker)
            
            if fit_orientation: 
                z_axis = observed_axes[2]
                marker = hr.create_axis_marker(xyz, z_axis, marker_id, self.marker_frame, self.marker_time, self.rgba)
                marker_id += 1
                ros_markers.append(marker)
                
            if visualize_targets:
                target_rgba = [1.0, 1.0, 1.0, 1.0]
                xyz = target_xyz
                marker = hr.create_sphere_marker(xyz, marker_id, self.marker_frame, self.marker_time, target_rgba)
                marker_id += 1
                ros_markers.append(marker)

                if fit_orientation: 
                    z_axis = target_axes[2]
                    marker = hr.create_axis_marker(xyz, z_axis, marker_id, self.marker_frame, self.marker_time, target_rgba)
                    marker_id += 1
                    ros_markers.append(marker)

        # Returns the error dictionary, a list of ROS markers for
        # visualization, and an ID number for the last ROS marker
        # generated.
        return error, ros_markers, marker_id
