#!/usr/bin/env python


import roslib
import rospy
import tf
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
import time
import pyrealsense2
from cv_bridge import CvBridge, CvBridgeError


def convert_depth_image(self, ros_image):
    bridge = CvBridge()
    try:
        depth_image = cv_bridge.imgmsg_to_cv2(ros_image, deside_encoding="passthrough")
        depth_array = np.array(depth_image, dtype=np.float32)
        rospy.loginfo(depth_array)
    except CvBridgeError:
        print('CvBridge Error')
def pixel2depth():
    rospy.init_node('pixel2depth',anonymous=True)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=convert_depth_image, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    pixel2depth()