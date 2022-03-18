#! /usr/bin/python3

from sklearn.cluster import k_means
import rospy
import numpy as np
from vz_acoustic_scene_analysis.msg import MyAudioData
from std_msgs.msg import String
from scipy.io.wavfile import write
from collections import deque
import time
from pathlib import Path



# Non ROS import
# import acoustic_scene_analysis as asa

class RosInterface:
    def __init__(self):
        home_dir = str(Path.home())
        self.save_dir = home_dir + "/Music/"
        # 1) Write subscriber to /audio topic and populate a numpy data structure (array) with the uint8[] data
        # self.maxSize = 7
        # self.queue = [None] * 7
        # self.head = self.tail = -1
        self.wav_data = []
        self.arraylength = 0
        self.msg_count = 0

        rospy.Subscriber("/audio", MyAudioData, self.raw_callback)

    def enqueue(self,data):
        # if queue is full
        if ((self.tail+1) % self.k == self.head):
            # convert to mp3
            # publish mp3
            # remove the first element (call dequeue)
            pass
        elif (self.head == -1):
            self.head = 0
            self.tail = 0
        else:
            self.tail = (self.tail +1) % self.maxSize
            self.queue[self.tail] = data

    def dequeue(self):
        # if empty queue
        if (self.head == -1):
            pass
        # if the self
        else:
            temp = self.queue[self.head]

    def raw_callback(self, msg):
        # print("Length of uint8[]:", len(msg.data))
        self.wav_data.append(msg.data)

        # if (self.msg_count < 10000):
        #     self.arraylength += len(msg.data)
        #     print(self.nparray)
        #     print(len(bytes))
        # else :
        #     self.byteArray[self.msg_count] = bytes
        #     print(len(bytes))
        self.msg_count += 1 

    def on_shutdown(self):
        wav_arr = np.array(self.wav_data)
        print(wav_arr)
        print(wav_arr.shape)
        write(self.save_dir +'test.mp3', 44100, wav_arr)
        print("check music")
        pass

# 2) Check you're "Decoding" the audio ROS message correctly by saving to a .wav file
# 3) Be curious. Listen to the .wav file and see if it sounds gucci, and then maybe twiddle about with the encoding on the audio_capture.cpp, seeing if that changes anything e.g. encoding with mp3 instead.
# 4) If you've made it this far, well done. Try find Utku's function to pass the np array.


if __name__ == '__main__':
    try:
        rospy.init_node('ros_interface', anonymous=True)
        ros_int = RosInterface()

        rospy.on_shutdown(ros_int.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Audio converter node failed!')
        pass