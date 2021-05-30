#!/usr/bin/env python3

import cv2
import sys
import rospy
import object_detector as od
import detection_node as dn
import deep_learning_model_options as do
        
if __name__ == '__main__':    
    print('cv2.__version__ =', cv2.__version__)
    print('Python version (must be > 3.0):', sys.version)
    assert(int(sys.version[0]) >= 3)

    
    models_directory = do.get_directory()
    print('Using the following directory for deep learning models:', models_directory)        
    use_neural_compute_stick = do.use_neural_compute_stick()
    if use_neural_compute_stick:
        print('Attempt to use an Intel Neural Compute Stick 2.')
    else:
        print('Not attempting to use an Intel Neural Compute Stick 2.')
    
    use_tiny = True
    if use_tiny:
        confidence_threshold = 0.0 
    else:
        confidence_threshold = 0.5

    detector = od.ObjectDetector(models_directory,
                                 use_tiny_yolo3=use_tiny,
                                 confidence_threshold=confidence_threshold,
                                 use_neural_compute_stick=use_neural_compute_stick)
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

