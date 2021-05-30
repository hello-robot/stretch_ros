#!/usr/bin/env python3

import sys
import glob
import head_estimator as he
import cv2
import deep_learning_model_options as do

def pix_xy(x_float, y_float):
    return (int(round(x_float)), int(round(y_float)))

if __name__ == '__main__':

    print('cv2.__version__ =', cv2.__version__)
    print('Python version =', sys.version)
    assert(int(sys.version[0]) >= 3)
    
    models_directory = do.get_directory()
    print('Using the following directory for deep learning models:', models_directory)        
    use_neural_compute_stick = do.use_neural_compute_stick()
    if use_neural_compute_stick:
        print('Attempt to use an Intel Neural Compute Stick 2.')
    else:
        print('Not attempting to use an Intel Neural Compute Stick 2.')
    
    only_display_result_images = True

    input_dir = './test_images/'
    output_dir = './output_images/'
    filenames = glob.glob(input_dir + '*')
    filenames.sort()

    print('Will attempt to load the following files:')
    for f in filenames:
        print(f)
    
    estimator = he.HeadPoseEstimator(models_directory,
                                     use_neural_compute_stick=use_neural_compute_stick)
    
    for i, f in enumerate(filenames): 
        rgb_image = cv2.imread(f)
        if rgb_image is not None:
            heads, output_image = estimator.apply_to_image(rgb_image, draw_output=True)
            cv2.imwrite(output_dir + 'face_detection_and_pose_estimation_' + str(i) + '.png', output_image)
