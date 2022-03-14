#! /usr/bin/python3

import rospy
import numpy as np
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
from scipy.io.wavfile import wavf

class D435i:
    def __init__(self):
        # 1) Write subscriber to /audio topic and populate a numpy data structure (array) with the uint8[] data
        self.raw_audio_sub = rospy.Subscriber('/audio', AudioData, self.raw_callback)

        self.byteArray = np.array(1)
        self.msg_count = 0

    def raw_callback(self, bytes):
        if (self.msg_count > 0):
            self.byteArray.append(bytes)
        else :
            self.byteArray()
        self.msg_count += 1 

# 2) Check you're "Decoding" the audio ROS message correctly by saving to a .wav file
# 3) Be curious. Listen to the .wav file and see if it sounds gucci, and then maybe twiddle about with the encoding on the audio_capture.cpp, seeing if that changes anything e.g. encoding with mp3 instead.
# 4) If you've made it this far, well done. Try find Utku's function to pass the np array.

    def run(self):
        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':
    try:
        rospy.init_node('audio_converter', anonymous=True)
    except rospy.ROSInterruptException:
        print('Audio converter node failed!')
        pass