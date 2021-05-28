#!/usr/bin/env python3

import cv2
import numpy as np

import rospy

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation

import hello_helpers.fit_plane as fp
import hello_helpers.hello_ros_viz as hr


class DetectionBoxMarker:
    def __init__(self, detection_box_id, marker_base_name):
        self.detection_box_id = 4 * detection_box_id
        
        colormap = cv2.COLORMAP_HSV
        offset = 0
        i = (offset + (self.detection_box_id * 29)) % 255
        image = np.uint8([[[i]]])
        id_color_image = cv2.applyColorMap(image, colormap)
        bgr = id_color_image[0,0]
        self.id_color = [bgr[2], bgr[1], bgr[0]]
        
        self.frame_id = 'camera_color_optical_frame'

        self.name = marker_base_name
        
        self.marker = Marker()
        self.marker.type = self.marker.CUBE
        self.marker.action = self.marker.ADD
        self.lifetime_s = 2.0
        self.marker.lifetime = rospy.Duration(self.lifetime_s)
        # although useful, this causes a warning and rviz and goes
        # against the documentation "NOTE: only used for text markers
        # string text"
        self.marker.text = self.name

        self.frame_number = None
        self.timestamp = None
        self.plane = None
        self.points_array = None
        self.ready = False

        self.box_3d = None
        self.x_axis = None
        self.y_axis = None
        self.z_axis = None
        
        self.detection_box_width_m = None
        self.detection_box_height_m = None

        self.landmarks_xyz = None
        self.depth = None

        
    def get_marker_point_cloud(self):
        return self.points_array
    
    def get_plane_fit_point_cloud(self):
        if self.plane is None:
            return None
        origin = np.array(self.marker_position)
        side_length = max(self.detection_box_width_m, self.detection_box_height_m)
        sample_spacing = 0.001
        points = self.plane.get_points_on_plane(origin, side_length, sample_spacing)
        return points
    
    def update(self, detection_3d, timestamp, frame_number):

        self.timestamp = timestamp
        self.frame_number = frame_number
        self.points_array = detection_3d['points_3d']
        self.landmarks_xyz = detection_3d['landmarks_3d']
        
        self.box_3d = detection_3d['box_3d']
        if self.box_3d is not None: 
            self.marker_position = self.box_3d['center_xyz']
            self.marker_quaternion = self.box_3d['quaternion']
            self.x_axis = self.box_3d['x_axis']
            self.y_axis = self.box_3d['y_axis']
            self.z_axis = self.box_3d['z_axis']
            self.detection_box_width_m = self.box_3d['width_m']
            self.detection_box_height_m = self.box_3d['height_m']
            plane = self.box_3d['plane']
            if plane is not None:
                n = plane['n']
                d = plane['d']
                self.plane = fp.FitPlane()
                self.plane.set_plane(n,d)
        
        self.ready = True


    def get_landmarks_marker(self, landmark_color_dict=None):
        marker = None
        if self.landmarks_xyz is not None:
            id_num = (4 * self.detection_box_id) + 3
            marker = hr.create_points_marker(self.landmarks_xyz, id_num,
                                             self.frame_id, self.timestamp,
                                             points_rgba=landmark_color_dict,
                                             duration_s=self.lifetime_s,
                                             point_width=0.02)
        return marker

    
    def get_ros_marker(self):
        if (not self.ready) or (self.box_3d is None):
            return None

        self.marker.header.frame_id = self.frame_id
        self.marker.header.stamp = self.timestamp
        self.marker.id = self.detection_box_id

        # scale of 1,1,1 would result in a 1m x 1m x 1m cube
        self.marker.scale.x = self.detection_box_width_m
        self.marker.scale.y = self.detection_box_height_m
        self.marker.scale.z = 0.01 # 1 cm tall

        # make as bright as possible
        den = float(np.max(self.id_color))
        self.marker.color.r = self.id_color[2]/den
        self.marker.color.g = self.id_color[1]/den
        self.marker.color.b = self.id_color[0]/den
        self.marker.color.a = 0.5

        self.marker.pose.position.x = self.marker_position[0]
        self.marker.pose.position.y = self.marker_position[1]
        self.marker.pose.position.z = self.marker_position[2]

        q = self.marker_quaternion
        self.marker.pose.orientation.x = q[0]
        self.marker.pose.orientation.y = q[1]
        self.marker.pose.orientation.z = q[2]
        self.marker.pose.orientation.w = q[3]        

        return self.marker


    def create_axis_marker(self, axis, id_num, rgba=None, name=None, axes_scale=1.0): 
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.timestamp
        marker.id = id_num
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.lifetime = rospy.Duration(1.0)
        if name is not None:
            # although useful, this causes a warning and rviz and goes
            # against the documentation "NOTE: only used for text markers
            # string text"
            marker.text = name
        # axis_arrow = {'head_diameter': 0.02,
        #               'shaft_diameter': 0.012,
        #               'head_length': 0.012, 
        #               'length': 0.08}

        axis_arrow = {'head_diameter': 0.02 * axes_scale,
                      'shaft_diameter': 0.012 * axes_scale,
                      'head_length': 0.012 * axes_scale, 
                      'length': 0.08 * axes_scale}

        
        # "scale.x is the shaft diameter, and scale.y is the
        # head diameter. If scale.z is not zero, it specifies
        # the head length." -
        # http://wiki.ros.org/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29
        marker.scale.x = axis_arrow['shaft_diameter']
        marker.scale.y = axis_arrow['head_diameter']
        marker.scale.z = axis_arrow['head_length']

        if rgba is None: 
            # make as bright as possible
            den = float(np.max(self.id_color))
            marker.color.r = self.id_color[2]/den #1.0
            marker.color.g = self.id_color[1]/den #0.0
            marker.color.b = self.id_color[0]/den #0.0
            marker.color.a = 0.5
        else:
            c = marker.color
            c.r, c.g, c.b, c.a = rgba

        start_point = Point()
        x = self.marker_position[0]
        y = self.marker_position[1]
        z = self.marker_position[2]
        start_point.x = x
        start_point.y = y
        start_point.z = z
        end_point = Point()
        length = axis_arrow['length']
        end_point.x = x + (axis[0] * length)
        end_point.y = y + (axis[1] * length)
        end_point.z = z + (axis[2] * length)
        marker.points = [start_point, end_point]
        return marker
    
    def get_ros_z_axis_marker(self):
        if (not self.ready) or (self.z_axis is None):
            return None

        id_num = 4 * self.detection_box_id
        rgba = [0.0, 0.0, 1.0, 0.5]
        name = base_name = '_z_axis'
        return self.create_axis_marker(self.z_axis, id_num, rgba, name)
        
    def get_ros_axes_markers(self, axes_scale=1.0):
        markers = []
        
        if not self.ready:
            return markers

        # ROS color convention
        # x axis is red
        # y axis is green
        # z axis is blue

        base_name = self.name
        
        if self.z_axis is not None:
            id_num = 4 * self.detection_box_id
            rgba = [0.0, 0.0, 1.0, 0.5]
            name = base_name = '_z_axis'
            markers.append(self.create_axis_marker(self.z_axis, id_num, rgba, name, axes_scale=axes_scale))
        if self.x_axis is not None:
            id_num = (4 * self.detection_box_id) + 1
            rgba = [1.0, 0.0, 0.0, 0.5]
            name = base_name = '_x_axis'
            markers.append(self.create_axis_marker(self.x_axis, id_num, rgba, name, axes_scale=axes_scale))
        if self.y_axis is not None:
            id_num = (4 * self.detection_box_id) + 2
            rgba = [0.0, 1.0, 0.0, 0.5]
            name = base_name = '_y_axis'
            markers.append(self.create_axis_marker(self.y_axis, id_num, rgba, name, axes_scale=axes_scale))
        
        return markers
    
            
        
class DetectionBoxMarkerCollection:
    def __init__(self, default_marker_base_name='detection_box'):
        self.collection = {}
        self.frame_number = 0
        self.default_marker_base_name = default_marker_base_name
        
    def __iter__(self):
        # iterates through currently visible DetectionBox markers
        keys = self.collection.keys()
        for k in keys:
            marker = self.collection[k]
            if marker.frame_number == self.frame_number:
                yield marker

    def update(self, detections_3d, timestamp=None):
        self.frame_number += 1
        self.timestamp = timestamp

        self.collection.clear()
        self.detection_box_id = 0
        
        for detection_3d in detections_3d:
            box_3d = detection_3d['box_3d']
            landmarks_3d = detection_3d['landmarks_3d']
            label = detection_3d['label']
            
            if (box_3d is not None) or (landmarks_3d is not None):
                self.detection_box_id += 1
                if label is None:
                    marker_label = self.default_marker_base_name
                else:
                    marker_label = label
                new_marker = DetectionBoxMarker(self.detection_box_id, marker_label)
                self.collection[self.detection_box_id] = new_marker
                self.collection[self.detection_box_id].update(detection_3d, self.timestamp, self.frame_number)
                
                
    def get_ros_marker_array(self, landmark_color_dict=None):
        marker_array = MarkerArray()        
        for key in self.collection:
            marker = self.collection[key]
            if marker.frame_number == self.frame_number:
                ros_marker = marker.get_ros_marker()
                if ros_marker is not None: 
                    marker_array.markers.append(ros_marker)

                landmarks_marker = marker.get_landmarks_marker(landmark_color_dict)
                if landmarks_marker is not None: 
                    marker_array.markers.append(landmarks_marker)
        return marker_array

    def get_ros_axes_array(self, include_z_axes=True, include_axes=True, axes_scale=1.0):
        marker_array = MarkerArray()        
        for key in self.collection:
            marker = self.collection[key]
            if marker.frame_number == self.frame_number:
                if include_z_axes:
                    ros_z_axis_marker = marker.get_ros_z_axis_marker()
                    if ros_z_axis_marker is not None:
                        marker_array.markers.append(ros_z_axis_marker)
                if include_axes:
                    ros_axes_markers= marker.get_ros_axes_markers(axes_scale=axes_scale)
                    marker_array.markers.extend(ros_axes_markers)
        return marker_array
