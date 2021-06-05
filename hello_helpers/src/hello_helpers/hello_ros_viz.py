
#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np

def create_line_strip(points, id_num, frame_id, timestamp, rgba=[1.0, 0.0, 0.0, 1.0], line_width_m=0.01, duration_s=0.0):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = timestamp
    marker.id = id_num
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(duration_s)

    # scale of 1,1,1 would result in a 1m x 1m x 1m cube

    # "Line strips also have some special handling for scale: only
    # scale.x is used and it controls the width of the line segments."
    # http://wiki.ros.org/rviz/DisplayTypes/Marker#Line_Strip_.28LINE_STRIP.3D4.29
    marker.scale.x = line_width_m

    c = marker.color
    c.r, c.g, c.b, c.a = rgba
    
    marker.points = []
    for p in points:
        ros_p = Point()
        ros_p.x = p[0]
        ros_p.y = p[1]
        ros_p.z = p[2]
        marker.points.append(ros_p)
    return marker
        
def create_sphere_marker(xyz, id_num, frame_id, timestamp, rgba=[1.0, 0.0, 0.0, 1.0], diameter_m=0.01, duration_s=0.0):
    # a marker duration of 0 should last forever
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = timestamp
    marker.id = id_num
    marker.type = marker.SPHERE #CUBE
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(duration_s)
    # scale of 1,1,1 would result in a 1m x 1m x 1m cube
    marker.scale.x = diameter_m
    marker.scale.y = diameter_m
    marker.scale.z = diameter_m
    marker.pose.position.x = xyz[0]
    marker.pose.position.y = xyz[1]
    marker.pose.position.z = xyz[2]
    c = marker.color
    c.r, c.g, c.b, c.a = rgba
    return marker

def create_axis_marker(xyz, axis, id_num, frame_id, timestamp, rgba, length=0.02, duration_s=0.0, arrow_scale=1.0): 
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = timestamp
    marker.id = id_num
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(duration_s)
    axis_arrow = {'head_diameter': (arrow_scale * 0.005),
                  'shaft_diameter': (arrow_scale * 0.003),
                  'head_length': (arrow_scale * 0.003)}
    # "scale.x is the shaft diameter, and scale.y is the
    # head diameter. If scale.z is not zero, it specifies
    # the head length." -
    # http://wiki.ros.org/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29
    marker.scale.x = axis_arrow['shaft_diameter']
    marker.scale.y = axis_arrow['head_diameter']
    marker.scale.z = axis_arrow['head_length']
    c = marker.color
    c.r, c.g, c.b, c.a = rgba
    start_point = Point()
    x, y, z = xyz
    start_point.x = x
    start_point.y = y
    start_point.z = z
    end_point = Point()
    end_point.x = x + (axis[0] * length)
    end_point.y = y + (axis[1] * length)
    end_point.z = z + (axis[2] * length)
    marker.points = [start_point, end_point]
    return marker

def create_points_marker(points_xyz, id_num, frame_id, timestamp,
                         points_rgba=None, duration_s=0.0,
                         default_rgba=(0.0, 1.0, 0.0, 1.0),
                         point_width=0.005): 
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = timestamp
    marker.id = id_num
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(duration_s)

    # ROS documentation: "scale.x is point width, scale.y is point
    # height"
    marker.scale.x = point_width
    marker.scale.y = point_width
    
    points = []
    colors = []
    for name, xyz in points_xyz.items(): 
        p = Point()
        x, y, z = xyz
        p.x = x
        p.y = y
        p.z = z
        points.append(p)
        c = ColorRGBA()
        if points_rgba is None: 
            c.r, c.g, c.b, c.a = default_rgba
        else:
            c.r, c.g, c.b, c.a = points_rgba[name]
        colors.append(c)
            
    marker.points = points
    marker.colors = colors
    return marker

