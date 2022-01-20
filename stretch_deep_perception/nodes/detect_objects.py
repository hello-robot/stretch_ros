#!/usr/bin/env python3

import cv2
import sys
import rospy

import object_detect_pytorch as od
import detection_node as dn
import deep_learning_model_options as do
        
if __name__ == '__main__':
    confidence_threshold = 0.0
    detector = od.ObjectDetector(confidence_threshold=confidence_threshold)
    default_marker_name = 'object'
    node_name = 'DetectObjectsNode'
    topic_base_name = 'objects'
    fit_plane = False
    node = dn.DetectionNode(detector, default_marker_name, node_name, topic_base_name, fit_plane)
    node.main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')

