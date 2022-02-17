#!/usr/bin/env python

# Goal Aruco Marker Detection subscribe
import rospy
import actionlib
import math
import time
import threading
import sys
import argparse as ap
import numpy as np
import ros_numpy
import yaml
import time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from collect_head_calibration_data import CollectHeadCalibrationDataNode


if __name__ == '__main__':    
    parser = ap.ArgumentParser(description='Collect Test Rig data.')
    parser.add_argument('--check', action='store_true', help='Collect data to check the current calibration, instead of data to perform a new calibration.')
    
    args, unknown = parser.parse_known_args()
    collect_check_data = args.check
     
    try:
        node = CollectHeadCalibrationDataNode(test_rig=True)
        node.number_of_testrig_samples = 200
        node.main(collect_check_data)
    except rospy.ROSInterruptException:
        pass