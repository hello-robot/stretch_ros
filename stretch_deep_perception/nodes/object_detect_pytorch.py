#!/usr/bin/env python3

import cv2
import numpy as np
import torch
import pandas
import ros_numpy

import deep_models_shared_python3 as dm


class ObjectDetector:
    def __init__(self, confidence_threshold=0.2):
        # Load the models
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom
        self.confidence_threshold = confidence_threshold
        
    def get_landmark_names(self):
        return None

    def get_landmark_colors(self):
        return None

    def get_landmark_color_dict(self):
        return None
    
    def apply_to_image(self, rgb_image, draw_output=False):
        results = self.model(rgb_image)
        object_detections = results.pandas().xyxy[0]

        results = []
        for index, detection in object_detections.iterrows():
            confidence = detection['confidence']
            if confidence > self.confidence_threshold:
                class_label = detection['name']
                object_class_id = detection['class']
                x_min = detection['xmin']
                x_max = detection['xmax']
                y_min = detection['ymin']
                y_max = detection['ymax']
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
