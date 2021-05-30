#!/usr/bin/env python3

import cv2
import sys
import rospy
import numpy as np
import head_estimator as he
import detection_node as dn
import deep_learning_model_options as do

def faces_3d_to_nearest_mouth_position(detections_3d):
    for d in detections_3d:
        landmarks_3d = d['landmarks_3d']
        box_3d = d['box_3d']
        if landmarks_3d is not None:
            width_vec = np.array(landmarks_3d['mouth_left']) - np.array(landmarks_3d['mouth_right'])
            width_m = np.linalg.norm(width_vec)
            height_vec = np.array(landmarks_3d['mouth_top']) - np.array(landmarks_3d['mouth_bottom'])
            height_m = np.linalg.norm(height_vec)

            mouth_landmarks = ['mouth_right', 'mouth_left', 'mouth_top', 'mouth_bottom']
            mouth_sum = np.array([0.0, 0.0, 0.0])
            for mouth_point in mouth_landmarks:
                mouth_sum += np.array(landmarks_3d[mouth_point])
            mouth_center = mouth_sum / 4.0
            
            if box_3d is not None:
                box_3d['width_m'] = width_m
                box_3d['height_m'] = height_m
                z_axis = np.array(box_3d['z_axis'])
                z_axis = z_axis / np.linalg.norm(z_axis)
                safety_m = 0.0 #-0.05
                new_center = mouth_center + (safety_m * z_axis)
                box_3d['center_xyz'] = (new_center[0], new_center[1], new_center[2])
    nearest_mouth_detection = None
    nearest_mouth_depth = None
    for d in detections_3d:
        box_3d = d['box_3d']
        if box_3d is not None: 
            if nearest_mouth_detection is None: 
                nearest_mouth_detection = d
                nearest_mouth_depth = box_3d['center_xyz'][2]
            else:
                depth = box_3d['center_xyz'][2]
                if depth < nearest_mouth_depth:
                    nearest_mouth_detection = d
                    nearest_mouth_depth = box_3d['center_xyz'][2]
    if nearest_mouth_detection is not None:
        detections_3d = [nearest_mouth_detection]
    else:
        detections_3d = []
    return detections_3d


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

    detector = he.HeadPoseEstimator(models_directory,
                                    use_neural_compute_stick=use_neural_compute_stick)
    default_marker_name = 'mouth'
    node_name = 'DetectFacesNode'
    topic_base_name = 'nearest_mouth'
    fit_plane = False
    node = dn.DetectionNode(detector,
                            default_marker_name,
                            node_name,
                            topic_base_name,
                            fit_plane,
                            modify_3d_detections=faces_3d_to_nearest_mouth_position,
                            min_box_side_m=min_head_m,
                            max_box_side_m=max_head_m)
    node.main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')
