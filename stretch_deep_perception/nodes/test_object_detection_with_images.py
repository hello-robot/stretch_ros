#!/usr/bin/env python3

import sys
import glob
import object_detector as jd
import cv2
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

    use_tiny = True
    if use_tiny:
        confidence_threshold = 0.0
    else:
        confidence_threshold = 0.5
        
    object_detector = jd.ObjectDetector(models_directory,
                                        use_tiny_yolo3=use_tiny,
                                        use_neural_compute_stick=use_neural_compute_stick)
    
    for i, f in enumerate(filenames): 
        rgb_image = cv2.imread(f)
        if rgb_image is not None:
            output_image = rgb_image.copy()
            results, output_image = object_detector.apply_to_image(rgb_image, draw_output=True)
            out_filename = output_dir + 'object_detection_' + str(i) + '.png'
            print('writing', out_filename)
            cv2.imwrite(out_filename, output_image)

