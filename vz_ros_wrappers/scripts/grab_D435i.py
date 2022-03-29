#! /usr/bin/python3
'''
Derived this from Nathaniel Hanson : https://github.com/RIVeR-Lab/flir_lepton
Purpose: Capture images from D435i and republish. This is only for sim. Stretch has its own driver
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

class D435i:
    def __init__(self):
        # Intialize empty stores for current image
        self.D435i_image = None
        # The following image will be flipped or not based on parameter /flip_images_grab_data
        self.D4355i_image_for_display = None

        # To flip or not to flip..
        self.flip_images_grab_data = rospy.get_param('flip_images_grab_data')

        # setup subscribed topics
        # initialize ros/cv2 bridge
        
        # Note: when testing with your own video, replace with local file
        self.D435i_cap = cv2.VideoCapture('/dev/video4')
        # self.D435i_cap = cv2.VideoCapture('/home/ananya/Downloads/2022-02-09-16-20-58(2).mp4')

        # Get topic parameter
        self.rgbd_topic = rospy.get_param("rgbd_topic")

        if not self.D435i_cap.isOpened():
            raise(Exception,'Unable to open video stream')
        self.bridge = CvBridge()
        self.D435i_pub = rospy.Publisher(self.rgbd_topic, Image, queue_size=100)
        self.timer = rospy.Timer(rospy.Duration(0, UPDATE_RATE), self.publish_image)
        self.timer_display = rospy.Timer(rospy.Duration(0, UPDATE_RATE), self.display_images)

    def publish_image(self, timer):
        try:
            if self.D435i_image is None:
                return

            img_msg = self.bridge.cv2_to_imgmsg(self.D435i_image,encoding="passthrough")
            img_msg.header.frame_id = 'D435i'
            img_msg.header.stamp = rospy.Time.now()

            self.D435i_pub.publish(img_msg)

        except Exception as e:
            print(traceback.print_exc())

    # todo ananya: comment out function below later once fully ros integrated
    def display_images(self, timer):
        if self.D435i_image is None:
            return
        try:
            # Show images
            if(self.flip_images_grab_data == True):
                ##### NOTE: Change rotation based on what you see ########
                self.D4355i_image_for_display = cv2.rotate(self.D435i_image,cv2.ROTATE_90_CLOCKWISE)
                ################################################################
            else:
                self.D4355i_image_for_display = self.D435i_image
            
            cv2.imshow('D435i',cv2.resize(self.D4355i_image_for_display,(600,800), interpolation = cv2.INTER_AREA))
            cv2.waitKey(3)

        except Exception as e:
            rospy.logwarn(e)

    def run(self):
        while not rospy.is_shutdown():
            ret, self.D435i_image = self.D435i_cap.read()
            
            # ##### NOTE: Change rotation based on what you see ########
            # self.D435i_image = cv2.rotate(self.D435i_image,cv2.ROTATE_90_CLOCKWISE)
            # ################################################################
            
            # Note : To read the input at the same frame rate as the recorded video, when using 
            # locally recorded video, uncomment the following and replace with (1/frame rate of video)
            # video_frame_rate_to_freq = 1/9
            # time.sleep(video_frame_rate_to_freq)

if __name__ == '__main__':
    try:
        rospy.init_node('D435i', anonymous=True)
        processor = D435i()
        processor.run()
    except rospy.ROSInterruptException:
        print('Image processing node failed!')
        pass
    cv2.destroyAllWindows()
