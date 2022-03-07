#! /usr/bin/python3
'''
Initial Author: Nathaniel Hanson
Date: 11/29/2021
File: grab_thermal.py

Purpose: Capture images from FLIR Lepton dev kit and republish
'''

# Future proofing python 2
from __future__ import nested_scopes
from __future__ import generators
from __future__ import division
from __future__ import absolute_import
from __future__ import with_statement
from __future__ import print_function
# Standard package imports
import rospy
import tf2_ros
import sys
import os
import cv2
import roslib
import math
import traceback
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

UPDATE_RATE = 33333333.333 # nanoseconds

class FLIR_LEPTON:
    def __init__(self):
        # Intialize empty stores for current image
        self.thermal_image = None

        # setup subscribed topics
        # initialize ros/cv2 bridge
        
        # the following depends on usb port we plug into
        # for the vz project stretch robot, the top aux usb is video6
        # Note: when testing with your own video, replace with local file

        self.thermal_cap = cv2.VideoCapture('/home/ananya/Downloads/Thermal-3(2).mp4')
        # self.thermal_cap = cv2.VideoCapture('/dev/video6')
        
        if not self.thermal_cap.isOpened():
            raise(Exception,'Unable to open video stream')
        self.bridge = CvBridge()
        self.thermal_pub = rospy.Publisher('/flir_3_5_near_realsense_raw', Image, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(0, UPDATE_RATE), self.publish_image)
        # self.timer_display = rospy.Timer(rospy.Duration(0, UPDATE_RATE), self.display_images)

    def publish_image(self, timer):
        try:
            if self.thermal_image is None:
                return
                
            img_msg = self.bridge.cv2_to_imgmsg(self.thermal_image, encoding="passthrough")
            img_msg.header.frame_id = 'flir_3_5_near_realsense'
            img_msg.header.stamp = rospy.Time.now()
            self.thermal_pub.publish(img_msg)
            
        except Exception as e:
            print(traceback.print_exc())

    # todo ananya: comment out function below later once fully ros integrated
    def display_images(self, timer):
        if self.thermal_image is None:
            return
        try:
            # Show images
            cv2.imshow('Lepton',cv2.resize(self.thermal_image,(600,800), interpolation = cv2.INTER_AREA))
            cv2.waitKey(3)

        except Exception as e:
            rospy.logwarn(e)

    def run(self):
        while not rospy.is_shutdown():
            ret, self.thermal_image = self.thermal_cap.read()
            # Note : To read the input at the same frame rate as the recorded video, when using 
            # locally recorded video, uncomment the following and replace with (1/frame rate of video)
            # Comment the following two lines when running on the robot
            video_frame_rate_to_freq = 1/7
            time.sleep(video_frame_rate_to_freq)
           
if __name__ == '__main__':
    try:
        rospy.init_node('FLIRLepton', anonymous=True)
        processor = FLIR_LEPTON()
        processor.run()
    except rospy.ROSInterruptException:
        print('Image processing node failed!')
        pass
    cv2.destroyAllWindows()