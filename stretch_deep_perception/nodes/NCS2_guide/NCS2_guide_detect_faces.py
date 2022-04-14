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

    #### load model directory where all the deep perception models are stored   
    models_directory = do.get_directory()
    print('Using the following directory for deep learning models:', models_directory) 
    
    ######################################################
    # check if the neural compute stick is able to run the deep learning model. 
    # Not all models are compatile to run on NCS2.
    # Currently this only work for two models: object and face
    # detection.  Currently it does not work with head pose
    # estimation, facial landmarks, and body landmarks.
    # The reason is that NCS only supports models to be run through OpenVINO. 
    # There are OpenVINO format model zoos availble on the above mentioned models.
    # In case you want to run a model which is not availble in the model zoo, 
    # There is an API availabe to convert any format into OpenVINO format.
    # https://docs.openvino.ai/2021.2/openvino_docs_MO_DG_prepare_model_convert_model_Convert_Model_From_Caffe.html

    # The neural compute stick is called in deep_learning_model_options Refer that file for further instructions
    
    #####################################################


    use_neural_compute_stick = do.use_neural_compute_stick()
    if use_neural_compute_stick:
        print('Attempt to use an Intel Neural Compute Stick 2.')
    else:
        print('Not attempting to use an Intel Neural Compute Stick 2.')
    
    #  Each model has to have two formats one to run on nomal cpu which can be the direct caffe/pytorch/tensorflow etc., format
    # second is the OpenVINO format.
    # If the model is compatible to run on NCS 2, make sure to turn on the NCS feature by passing the use_neural_compute_stick.
    detector = he.HeadPoseEstimator(models_directory,
                                    use_neural_compute_stick=use_neural_compute_stick)

    ### The rest of the code is normal
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

