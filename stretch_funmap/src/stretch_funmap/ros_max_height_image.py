#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np
import math

import message_filters
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Transform, Pose, Vector3, Quaternion, Point

import ros_numpy

import struct
import threading
from collections import deque
        
import tf2_ros
from scipy.spatial.transform import Rotation

from copy import deepcopy

from stretch_funmap.max_height_image import *

import stretch_funmap.navigation_planning as na
    
class ROSVolumeOfInterest(VolumeOfInterest):


    @classmethod
    def from_serialization(self, data):
        d = data
        voi = ROSVolumeOfInterest(d['frame_id'], np.array(d['origin']), np.array(d['axes']), d['x_in_m'], d['y_in_m'], d['z_in_m'])
        return voi

    def get_voi_to_points_matrix_with_tf2(self, points_frame_id, tf2_buffer, lookup_time=None, timeout_s=None):
        points_to_voi_mat, timestamp = self.get_points_to_voi_matrix_with_tf2(points_frame_id, tf2_buffer, lookup_time=lookup_time, timeout_s=timeout_s)
        voi_to_points_mat = None
        if points_to_voi_mat is not None: 
            voi_to_points_mat = np.linalg.inv(points_to_voi_mat)
        return voi_to_points_mat, timestamp
    
    def get_points_to_voi_matrix_with_tf2(self, points_frame_id, tf2_buffer, lookup_time=None, timeout_s=None):
        # If the necessary TF2 transform is successfully looked up,
        # this returns a 4x4 affine transformation matrix that
        # transforms points in the points_frame_id frame to the voi
        # frame.
        try:
            if lookup_time is None:
                lookup_time = rospy.Time(0) # return most recent transform
            if timeout_s is None:
                timeout_ros = rospy.Duration(0.1)
            else:
                timeout_ros = rospy.Duration(timeout_s)
            stamped_transform =  tf2_buffer.lookup_transform(self.frame_id, points_frame_id, lookup_time, timeout_ros)
            points_to_frame_id_mat = ros_numpy.numpify(stamped_transform.transform)
            points_to_voi_mat = self.get_points_to_voi_matrix(points_to_frame_id_mat)
            return points_to_voi_mat, stamped_transform.header.stamp
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('ROSVolumeOfInterest.get_points_to_voi_matrix_with_tf2: failed to lookup transform. self.frame_id = {0}, points_frame_id = {1}, lookup_time = {2}, timeout_s = {3}'.format(self.frame_id, points_frame_id, lookup_time, timeout_s))
            return None, None
        
    def get_ros_marker(self, duration=0.2):
        marker = Marker()
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(duration)
        marker.text = 'volume of interest'

        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.id = 0

        # scale of 1,1,1 would result in a 1m x 1m x 1m cube
        marker.scale.x = self.x_in_m
        marker.scale.y = self.y_in_m
        marker.scale.z = self.z_in_m

        # make as bright as possible
        r, g, b = [0, 0, 255]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 0.25 

        # find the middle of the volume of interest
        center_vec = np.array([self.x_in_m/2.0, self.y_in_m/2.0, self.z_in_m/2.0])
        center = self.origin + np.dot(self.axes, center_vec)
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]

        q = Rotation.from_matrix(self.axes).as_quat()
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]        

        return marker

    
        
class ROSMaxHeightImage(MaxHeightImage):
    
    @classmethod
    def from_file( self, base_filename ):
        data, image, rgb_image, camera_depth_image = MaxHeightImage.load_serialization(base_filename)
                        
        m_per_pix = data['m_per_pix']
        m_per_height_unit = data['m_per_height_unit']
        image_origin = np.array(data['image_origin'])

        voi = ROSVolumeOfInterest.from_serialization(data['voi_data'])

        if camera_depth_image is not None:
            use_camera_depth_image = True
        else:
            use_camera_depth_image = False
        max_height_image = ROSMaxHeightImage(voi, m_per_pix, image.dtype, m_per_height_unit, use_camera_depth_image=use_camera_depth_image, image=image, rgb_image=rgb_image, camera_depth_image=camera_depth_image)
        max_height_image.rgb_image = rgb_image
        self.last_update_time = None

        return max_height_image


    def get_points_to_image_mat(self, ros_frame_id, tf2_buffer, lookup_time=None, timeout_s=None):
        # This returns a matrix that transforms a point in the
        # provided ROS frame to a point in the image. However, it does
        # not quantize the components of the image point, which is a
        # nonlinear operation that must be performed as a separate
        # step.
        points_to_voi_mat, timestamp = self.voi.get_points_to_voi_matrix_with_tf2(ros_frame_id, tf2_buffer, lookup_time=lookup_time, timeout_s=timeout_s)
        if points_to_voi_mat is not None:
            points_to_voi_mat[:3,3] = points_to_voi_mat[:3,3] - self.image_origin
            voi_to_image_mat = np.identity(4)
            voi_to_image_mat[0, 0] = 1.0/self.m_per_pix
            voi_to_image_mat[1, 1] = - 1.0/self.m_per_pix
            dtype = self.image.dtype
            if np.issubdtype(dtype, np.integer):
                voi_to_image_mat[2, 3] = 1.0
                voi_to_image_mat[2, 2] = 1.0 / self.m_per_height_unit
            else:
                rospy.logerr('ROSMaxHeightImage.get_points_to_image_mat: unsupported image type used for max_height_image, dtype = {0}'.format(dtype))
                assert(False)

            points_to_image_mat = np.matmul(voi_to_image_mat, points_to_voi_mat)
                
            if self.transform_original_to_corrected is not None: 
                points_to_image_mat = np.matmul(self.transform_original_to_corrected, points_to_image_mat)
                
            return points_to_image_mat, timestamp
        else:
            return None, None

        
    def get_image_to_points_mat(self, ros_frame_id, tf2_buffer, lookup_time=None, timeout_s=None):
        # This returns a matrix that transforms a point in the image
        # to a point in the provided ROS frame. However, it does not
        # quantize the components of the image point, which is a
        # nonlinear operation that would need to be performed as a
        # separate step.
        voi_to_points_mat, timestamp = self.voi.get_voi_to_points_matrix_with_tf2(ros_frame_id, tf2_buffer, lookup_time=lookup_time, timeout_s=timeout_s)
        if voi_to_points_mat is not None:
            image_to_voi_mat = np.identity(4)
            image_to_voi_mat[:3, 3] = self.image_origin
            image_to_voi_mat[0, 0] = self.m_per_pix
            image_to_voi_mat[1, 1] = -self.m_per_pix
            dtype = self.image.dtype
            if np.issubdtype(dtype, np.integer):
                image_to_voi_mat[2, 3] = image_to_voi_mat[2,3] - self.m_per_height_unit
                image_to_voi_mat[2, 2] = self.m_per_height_unit
            else:
                rospy.logerr('ROSMaxHeightImage.get_image_to_points_mat: unsupported image type used for max_height_image, dtype = {0}'.format(dtype))
                assert(False)

            points_in_image_to_frame_id_mat = np.matmul(voi_to_points_mat, image_to_voi_mat)
            
            if self.transform_corrected_to_original is not None: 
                points_in_image_to_frame_id_mat = np.matmul(points_in_image_to_frame_id_mat, self.transform_corrected_to_original)

            return points_in_image_to_frame_id_mat, timestamp
        else:
            return None, None

        
    def get_robot_pose_in_image(self, tf2_buffer):
        robot_to_image_mat, timestamp = self.get_points_to_image_mat('base_link', tf2_buffer)
        r0 = np.array([0.0, 0.0, 0.0, 1.0])
        r0 = np.matmul(robot_to_image_mat, r0)[:2]
        r1 = np.array([1.0, 0.0, 0.0, 1.0])
        r1 = np.matmul(robot_to_image_mat, r1)[:2]
        robot_forward = r1 - r0
        robot_ang_rad = np.arctan2(-robot_forward[1], robot_forward[0])
        robot_xy_pix = r0
        return robot_xy_pix, robot_ang_rad, timestamp

    def get_point_in_image(self, xyz, xyz_frame_id, tf2_buffer):
        point_to_image_mat, timestamp = self.get_points_to_image_mat(xyz_frame_id, tf2_buffer)
        p = np.matmul(point_to_image_mat, np.array([xyz[0], xyz[1], xyz[2], 1.0]))[:3]
        return p

    def get_pix_in_frame(self, xyz_pix, xyz_frame_id, tf2_buffer):
        image_to_point_mat, timestamp = self.get_image_to_points_mat(xyz_frame_id, tf2_buffer)
        p = np.matmul(image_to_point_mat, np.array([xyz_pix[0], xyz_pix[1], xyz_pix[2], 1.0]))[:3]
        return p
    
    def make_robot_footprint_unobserved(self, robot_x_pix, robot_y_pix, robot_ang_rad):
        # replace robot points with unobserved points
        na.draw_robot_footprint_rectangle(robot_x_pix, robot_y_pix, robot_ang_rad, self.m_per_pix, self.image, value=0)
        if self.camera_depth_image is not None: 
            na.draw_robot_footprint_rectangle(robot_x_pix, robot_y_pix, robot_ang_rad, self.m_per_pix, self.camera_depth_image, value=0)
        if self.rgb_image is not None: 
            na.draw_robot_footprint_rectangle(robot_x_pix, robot_y_pix, robot_ang_rad, self.m_per_pix, self.rgb_image, value=0)


    def make_robot_mast_blind_spot_unobserved(self, robot_x_pix, robot_y_pix, robot_ang_rad):
        # replace mast blind spot wedge points with unobserved points
        na.draw_robot_mast_blind_spot_wedge(robot_x_pix, robot_y_pix, robot_ang_rad, self.m_per_pix, self.image, value=0)
        if self.camera_depth_image is not None: 
            na.draw_robot_mast_blind_spot_wedge(robot_x_pix, robot_y_pix, robot_ang_rad, self.m_per_pix, self.camera_depth_image, value=0)
        if self.rgb_image is not None: 
            na.draw_robot_mast_blind_spot_wedge(robot_x_pix, robot_y_pix, robot_ang_rad, self.m_per_pix, self.rgb_image, value=0)
    
    def from_points_with_tf2(self, points, points_frame_id, tf2_buffer, points_timestamp=None, timeout_s=None):
        # points should be a numpy array with shape = (N, 3) where N
        # is the number of points. So it has the following structure:
        # points = np.array([[x1,y1,z1], [x2,y2,z2]...]). The points
        # should be specified with respect to the coordinate system
        # defined by points_frame_id.
        
        points_to_voi_mat, timestamp = self.voi.get_points_to_voi_matrix_with_tf2(points_frame_id, tf2_buffer, lookup_time=points_timestamp, timeout_s=timeout_s)

        if points_to_voi_mat is not None: 
            self.from_points(points_to_voi_mat, points)

            if points_timestamp is None:
                if timestamp is None: 
                    self.last_update_time = rospy.Time.now()
                else:
                    self.last_update_time = timestamp
            else:
                self.last_update_time = points_timestamp
        else:
            rospy.logwarn('ROSMaxHeightImage.from_points_with_tf2: failed to update the image likely due to a failure to lookup the transform using TF2. points_frame_id = {0}, points_timestamp = {1}, timeout_s = {2}'.format(points_frame_id, points_timestamp, timeout_s))

    def from_rgb_points_with_tf2(self, rgb_points, points_frame_id, tf2_buffer, points_timestamp=None, timeout_s=None):
        # points should be a numpy array with shape = (N, 3) where N
        # is the number of points. So it has the following structure:
        # points = np.array([[x1,y1,z1], [x2,y2,z2]...]). The points
        # should be specified with respect to the coordinate system
        # defined by points_frame_id.
        
        points_to_voi_mat, timestamp = self.voi.get_points_to_voi_matrix_with_tf2(points_frame_id, tf2_buffer, lookup_time=points_timestamp, timeout_s=timeout_s)

        if points_to_voi_mat is not None: 
            self.from_rgb_points(points_to_voi_mat, rgb_points)

            if points_timestamp is None:
                if timestamp is None: 
                    self.last_update_time = rospy.Time.now()
                else:
                    self.last_update_time = timestamp
            else:
                self.last_update_time = points_timestamp
        else:
            rospy.logwarn('ROSMaxHeightImage.from_rgb_points_with_tf2: failed to update the image likely due to a failure to lookup the transform using TF2. points_frame_id = {0}, points_timestamp = {1}, timeout_s = {2}'.format(points_frame_id, points_timestamp, timeout_s))

            
    def to_point_cloud(self, color_map=None):
        points = self.to_points(color_map)
        point_cloud = ros_numpy.msgify(PointCloud2, points, stamp=self.last_update_time, frame_id=self.voi.frame_id)
        return point_cloud
