#! /usr/bin/python3

# ROS imports
from pickletools import uint8
from pprint import pp
from turtle import shape
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
        self.file_name = '/home/hello-robot/Desktop/bpm_vid.avi'
        self.fps = 30
        self.seconds = rospy.get_param("rec_secs")
        # self.frame_arr = np.empty(shape=(self.fps*self.seconds,2), dtype=uint8)
        self.frame_arr = np.empty(shape=300)
        # print(type(self.frame_arr))
        self.index = 0

        self.thermal_topic = rospy.get_param("thermal_topic")
        self.bpm_topic = rospy.get_param("bpm_topic")
        
        self.thermal_sub = rospy.Subscriber(self.thermal_topic, Image, self.thermal_cb)
        self.bpm_pub = rospy.Publisher(self.bpm_topic, Float64, queue_size=10)


    def thermal_cb(self, msg_data):
        cv_image = self.bridge.imgmsg_to_cv2(msg_data, desired_encoding='passthrough')
        print('CV_Image Type')
        print(type(cv_image[0,0,0]))
        print(cv_image.shape)
        for i in range(self.frame_arr):
            print(i)
            # self.frame_arr[i] = cv_image
            # if i == 299:
            #     pp_demo(self.frame_arr)


        # im_arr = np.asarray(cv_image)
        # if self.index == 0:
            # self.tzero = cv_image.header.stamp
        # self.frame_arr[self.index,0] = data.data
        # self.frame_arr[self.index,1] = data.header.stamp - self.tzero
        # self.index +=1
        
        # pp_demo.get_bpm(im_arr)
        # print(type(cv_image.data))
        # print(type(cv_image.header.stamp))

        #data.header.timestamp
        # img_arr = np.array(data.data, dtype=np.uint8)
        # # get image data into mp4 form
        # cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

        # if self.video_writer is None:
        #     rows, cols, _ = cv_image.shape
        #     fourcc = cv2.cv.CV_FOURCC(*'MJPG') # double check here -> cv2.VideoWriter_fourcc(*'MJPG')
        #     self.video_writer = cv2.VideoWriter(self.file_name, -1, self.fps, (cols, rows))
        
        # self.video_writer.write(cv_image)

    def cleanup(self):
        if self.video_writer is not None:
            self.video_writer.release()

        # for row in self.frame_arr:
        #     print(row[1])

if __name__== '__main__':
    try:
        rospy.init_node("bpm_capture")
        processor = FLIR_LEPTON()
        # processor.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('BPM extraction node failed!')
        rospy.on_shutdown(processor.cleanup)