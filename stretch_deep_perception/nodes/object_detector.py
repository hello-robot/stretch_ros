#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import deep_models_shared as dm


class ObjectDetector:
    def __init__(self, models_directory, use_tiny_yolo3 = True, confidence_threshold=0.2, use_neural_compute_stick=False):
        # Load the models
        models_dir = models_directory + 'darknet/'
        print('Using the following directory to load object detector models:', models_dir)
        
        if use_tiny_yolo3: 
            model_filename = models_dir + 'tiny_yolo_v3/yolov3-tiny.weights'
            config_filename = models_dir + 'tiny_yolo_v3/yolov3-tiny.cfg'
            classes_filename = models_dir + 'tiny_yolo_v3/object_detection_classes_yolov3.txt'
            input_width = 416 
            input_height = 416
        else:
            model_filename = models_dir + 'yolo_v3/yolov3.weights'
            config_filename = models_dir + 'yolo_v3/yolov3.cfg'
            classes_filename = models_dir + 'yolo_v3/object_detection_classes_yolov3.txt'
            input_width = 608
            input_height = 608
            
        self.input_width = input_width
        self.input_height = input_height

        self.confidence_threshold = confidence_threshold
        self.non_maximal_suppression = 0.01
        self.scale = 0.00392
        self.rgb = True
        self.mean = (0.0, 0.0, 0.0)

        if use_tiny_yolo3:
            print('using YOLO V3 Tiny')
        else:
            print('using YOLO V3')
                    
        print('models_dir =', models_dir)
        print('model_filename =', model_filename)
        print('config_filename =', config_filename)
        print('classes_filename =', classes_filename)

        classes_file = open(classes_filename, 'rt')
        raw_classes_text = classes_file.read()
        classes_file.close()
        self.object_class_labels = raw_classes_text.rstrip('\n').split('\n')
        self.num_object_classes = len(self.object_class_labels)
        self.object_detection_model = cv2.dnn.readNet(model_filename, config_filename, 'darknet')
        
        # attempt to use Neural Compute Stick 2
        if use_neural_compute_stick:
            print('ObjectDetector.__init__: Attempting to use an Intel Neural Compute Stick 2 using the following command: self.object_detection_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)')
            self.object_detection_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

        dm.print_model_info(self.object_detection_model, 'object_detection_model')
        
        self.output_layer_names = self.object_detection_model.getUnconnectedOutLayersNames()

    def get_landmark_names(self):
        return None

    def get_landmark_colors(self):
        return None

    def get_landmark_color_dict(self):
        return None
    
    def apply_to_image(self, rgb_image, draw_output=False):

        original_height, original_width, num_color = rgb_image.shape
        
        object_image_blob = cv2.dnn.blobFromImage(rgb_image,
                                                  1.0,
                                                  size=(self.input_width, self.input_height),
                                                  swapRB=self.rgb,
                                                  ddepth=cv2.CV_8U)
        
        self.object_detection_model.setInput(object_image_blob, scalefactor=self.scale, mean=self.mean)
        object_detections = self.object_detection_model.forward(self.output_layer_names)

        # object_detections is a list

        # YOLO v3 Tiny
        # object_detections = [ array with shape (507, 85),
        #                       array with shape (2028, 85) ]

        # YOLO v3
        # object_detections = [ array with shape (1083, 85),
        #                       array with shape (4332, 85),
        #                       array with shape (17328, 85) ]

        # each element of the list has a constant shape RxC
        
        # Each of the R rows represents a detection
        
        # [0:5] (the first 4 numbers) specify a bounding box
        # [box_center_x, box_center_y, box_width, box_height], where
        # each element is a scalar between 0.0 and 1.0 that can be
        # multiplied by the original input image dimensions to recover
        # the bounding box in the original image.

        # [5:] (the remaining 81 numbers) represent the confidence
        # that a particular class was detected in the bounding box (80
        # COCO object classes) plus one class that represents the
        # background and hence no detection (most likely - my
        # interpretation without really looking closely at it).

        def bound_x(x_in):
            x_out = max(x_in, 0)
            x_out = min(x_out, original_width - 1)
            return x_out

        def bound_y(y_in):
            y_out = max(y_in, 0)
            y_out = min(y_out, original_height - 1)
            return y_out

        results = []
        for detections in object_detections:
            object_class_confidences = detections[:,5:]
            best_object_classes = np.argmax(object_class_confidences, axis=1)

            # only consider non-background classes
            non_background_selector = best_object_classes < self.num_object_classes
            detected_objects = detections[non_background_selector]
            best_object_classes = best_object_classes[non_background_selector]

            # collect and prepare detected objects
            for detection, object_class_id in zip(detected_objects, best_object_classes):
                confidence = detection[5:][object_class_id]
                if confidence > self.confidence_threshold:
                    class_label = self.object_class_labels[object_class_id]

                    box_center_x, box_center_y, box_width, box_height = detection[:4]
                    
                    x_min = (box_center_x - (box_width / 2.0)) * original_width
                    y_min = (box_center_y - (box_height / 2.0)) * original_height
                    x_max = x_min + (box_width * original_width)
                    y_max = y_min + (box_height * original_height)

                    x_min = bound_x(int(round(x_min)))
                    y_min = bound_y(int(round(y_min)))
                    x_max = bound_x(int(round(x_max)))
                    y_max = bound_y(int(round(y_max)))
                    
                    box = (x_min, y_min, x_max, y_max)

                    print(class_label, ' detected')
                    
                    results.append({'class_id': object_class_id,
                                    'label': class_label,
                                    'confidence': confidence,
                                    'box': box})

        output_image = None
        if draw_output:
            output_image = rgb_image.copy()
            for detection_dict in results:
                self.draw_detection(output_image, detection_dict)
                    
        return results, output_image


    def draw_detection(self, image, detection_dict):
        font_scale = 0.75
        line_color = [0, 0, 0]
        line_width = 1
        font = cv2.FONT_HERSHEY_PLAIN
        class_label = detection_dict['label']
        confidence = detection_dict['confidence']
        box = detection_dict['box']
        x_min, y_min, x_max, y_max = box
        output_string = '{0}, {1:.2f}'.format(class_label, confidence)
        color = (0, 0, 255)
        rectangle_line_thickness = 2 #1
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, rectangle_line_thickness)

        # see the following page for a helpful reference
        # https://stackoverflow.com/questions/51285616/opencvs-gettextsize-and-puttext-return-wrong-size-and-chop-letters-with-low

        label_background_border = 2
        (label_width, label_height), baseline = cv2.getTextSize(output_string, font, font_scale, line_width)
        label_x_min = x_min
        label_y_min = y_min
        label_x_max = x_min + (label_width + (2 * label_background_border))
        label_y_max = y_min + (label_height + baseline + (2 * label_background_border))
        
        text_x = label_x_min + label_background_border
        text_y = (label_y_min + label_height) + label_background_border

        cv2.rectangle(image, (label_x_min, label_y_min), (label_x_max, label_y_max), (255, 255, 255), cv2.FILLED)
        cv2.putText(image, output_string, (text_x, text_y), font, font_scale, line_color, line_width, cv2.LINE_AA)
