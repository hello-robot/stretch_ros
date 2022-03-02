#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation
import math

x = np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]])

r1 =  [[-0.956395958000000,0.292073230000000,0.000014880000000],
       [-0.292073218000000,-0.956395931000000,0.000242173000000],
       [0.000084963000000,0.000227268000000,0.999999971000000]]

r2 = [[-0.956227882, 0.292623030000000, -0.000013768000000],
      [-0.292073218000000, -0.956227882000000,-0.000029806000000],
      [-0.000021887000000, 0.000024473000000, 0.999999999000000]]

print(x)

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
    r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_dcm()
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

def affine_matrix_difference(t1, t2, size=4):
    error = 0.0
    for i in range(size):
        for j in range(size):
            error += abs(t1[i,j] - t2[i,j])
    return error

def angle_rot_error(r1,r2):
    rot1 = np.array(r1)
    rot2 = np.array(r2)
    rot12 = np.matmul(rot1.T,rot2)
    theta_error = np.arccos((np.trace(rot12)-1)/2)
    return theta_error

print(np.rad2deg(angle_rot_error(r1,r2)))

print(x[:3,:3].shape)
