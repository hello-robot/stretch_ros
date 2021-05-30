#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import math
import glob
import body_landmark_detector as bl
import deep_learning_model_options as do


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

    detector = bl.BodyLandmarkDetector(models_directory,
                                       use_neural_compute_stick=use_neural_compute_stick)
        
    for i, f in enumerate(filenames):
        print('loading image =', f)
        rgb_image = cv2.imread(f)
        if rgb_image is not None:
            bodies, ignore = detector.apply_to_image(rgb_image)

            out_rgb = rgb_image.copy()
            print('bodies =', bodies)
            for body in bodies:
                print('body =', body)
                detector.draw_skeleton(out_rgb, body)

            cv2.imwrite(output_dir + 'skeleton_' + str(i) + '.png', out_rgb)
                
