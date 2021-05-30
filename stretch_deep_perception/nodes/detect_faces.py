#!/usr/bin/env python3

import cv2
import sys
import rospy
import head_estimator as he
import detection_node as dn
import deep_learning_model_options as do


if __name__ == '__main__':    
    print('cv2.__version__ =', cv2.__version__)
    print('Python version (must be > 3.0):', sys.version)
    assert(int(sys.version[0]) >= 3)
    
    ##############################################
    # Perform coarse filtering of 3D points using anthropometry
    #
    # 30cm should be significantly over the maximum dimensions of a human head
    # 10cm should be significantly smaller than the dimensions of an adult head
    # https://en.wikipedia.org/wiki/Human_head

    # children "attain 30% of adult head width by the middle of
    # prenatal life, 60% by birth, 80% by 6 postnatal months, 90%
    # by 3 years and 95% by 9 years" - GROWTH IN HEAD WIDTH DURING
    # THE FIRST TWELVE YEARS OF LIFE HOWARD V. MEREDITH, copyright
    # 1953 by the American Academy of Pediatrics
    # https://pediatrics.aappublications.org/content/12/4/411

    # Filtering for depths corresponding with heads with heights
    # or widths from 8cm to 40cm should be conservative.
    min_head_m = 0.08
    max_head_m = 0.4
    ##############################################
        
    models_directory = do.get_directory()
    print('Using the following directory for deep learning models:', models_directory)        
    use_neural_compute_stick = do.use_neural_compute_stick()
    if use_neural_compute_stick:
        print('Attempt to use an Intel Neural Compute Stick 2.')
    else:
        print('Not attempting to use an Intel Neural Compute Stick 2.')
    
    detector = he.HeadPoseEstimator(models_directory,
                                    use_neural_compute_stick=use_neural_compute_stick)
    default_marker_name = 'face'
    node_name = 'DetectFacesNode'
    topic_base_name = 'faces'
    fit_plane = False
    node = dn.DetectionNode(detector,
                            default_marker_name,
                            node_name,
                            topic_base_name,
                            fit_plane,
                            min_box_side_m=min_head_m,
                            max_box_side_m=max_head_m)
    node.main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')

