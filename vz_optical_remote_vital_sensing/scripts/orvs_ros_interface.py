#! /usr/bin/python3

# ROS imports
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

# Python imports
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2

# Custom imports
import VZ_ORVS.Thermal_Video_Submodule.Thermal_Camera_PostProcessing_Demo as pp_demo

class FLIR_LEPTON:
    def __init__(self):
        # Intialize empty stores for heart rate or breathes per min
        self.bpm = None

        self.bridge = CvBridge()
        self.video_writer = None
        self.file_name = 'bpm_vid.avi'
        self.fps = 20 # need to find actual value

        self.thermal_topic = rospy.get_param("thermal_topic")
        self.bpm_topic = rospy.get_param("bpm_topic")
        
        self.thermal_sub = rospy.Subscriber(self.thermal_topic, Image, self.thermal_cb)
        self.bpm_pub = rospy.Publisher(self.bpm_topic, Float64, queue_size=10)

    def thermal_cb(self, data):
        # get image data into mp4 form
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        if self.video_writer is None:
            rows, cols, _ = cv_image.shape
            self.video_writer = cv2.VideoWriter(self.file_name, -1, self.fps, (cols, rows))
        
        self.video_writer.write(cv_image)

    def cleanup(self):
        if self.video_writer is not None:
            self.video_writer.release()

if __name__== '__main__':
    try:
        rospy.init_node("bpm_capture")
        processor = FLIR_LEPTON()
        processor.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('BPM extraction node failed!')
        rospy.on_shutdown(processor.cleanup)