#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import deep_models_shared as dm

class BodyLandmarkDetector:
    def __init__(self, models_directory, confidence_threshold=0.2, landmarks_to_detect=None, use_neural_compute_stick=False):

        # Load the models
        model_dir = models_directory +'open_model_zoo/human-pose-estimation-0001/FP32/'
        print('Using the following directory to load body landmark models:', model_dir)
        skeleton_weights_filename = model_dir + 'human-pose-estimation-0001.bin'  
        skeleton_config_filename = model_dir + 'human-pose-estimation-0001.xml'
        print('Loading the following weights file:', skeleton_weights_filename)
        print('Loading the following config file:', skeleton_config_filename)

        self.skeleton_model = cv2.dnn.readNet(skeleton_weights_filename, skeleton_config_filename)
        
        # attempt to use Neural Compute Stick 2
        if use_neural_compute_stick:
            print('Not attempting to use a Intel Neural Compute Stick 2 for body landmarks due to potential errors.')
            
        dm.print_model_info(self.skeleton_model, 'skeleton_model')
        
        self.landmark_names = ['nose', 'neck',
                               'right_shoulder', 'right_elbow', 'right_wrist',
                               'left_shoulder', 'left_elbow', 'left_wrist',
                               'right_hip', 'right_knee', 'right_ankle',
                               'left_hip', 'left_knee', 'left_ankle',
                               'right_eye', 'left_eye',
                               'right_ear', 'left_ear']

        if landmarks_to_detect is None:
            self.default_landmarks_to_detect = self.landmark_names.copy()
        else:
            self.default_landmarks_to_detect = landmarks_to_detect

        # rgba
        self.landmark_color_dict = {'nose': (1.0, 1.0, 1.0, 1.0),
                                    'neck': (0.0, 1.0, 0.0, 1.0),
                                    'right_shoulder': (1.0, 0.0, 0.0, 1.0),
                                    'right_elbow': (0.0, 1.0, 0.0, 1.0),
                                    'right_wrist': (0.0, 0.0, 1.0, 1.0),
                                    'left_shoulder': (0.0, 0.0, 1.0, 1.0),
                                    'left_elbow': (0.0, 1.0, 0.0, 1.0),
                                    'left_wrist': (0.0, 0.0, 1.0, 1.0),
                                    'right_hip': (0.0, 1.0, 1.0, 1.0),
                                    'right_knee': (0.0, 1.0, 1.0, 1.0),
                                    'right_ankle': (0.0, 0.0, 1.0, 1.0),
                                    'left_hip': (0.0, 1.0, 1.0, 1.0),
                                    'left_knee': (0.0, 1.0, 1.0, 1.0),
                                    'left_ankle': (0.0, 0.0, 1.0, 1.0),
                                    'right_eye': (1.0, 0.0, 1.0, 1.0),
                                    'left_eye': (1.0, 0.0, 1.0, 1.0),
                                    'right_ear': (0.0, 1.0, 1.0, 1.0),
                                    'left_ear': (0.0, 1.0, 1.0, 1.0)}
        
        self.landmark_colors = [self.landmark_color_dict[n] for n in self.landmark_names]
        
        # based on COCO pairs defined in
        # https://github.com/opencv/opencv/blob/master/samples/dnn/openpose.py

        self.landmark_pairs = [['neck', 'right_shoulder'], ['neck', 'left_shoulder'],
                               ['right_shoulder', 'right_elbow'], ['right_elbow', 'right_wrist'],
                               ['left_shoulder', 'left_elbow'], ['left_elbow', 'left_wrist'],
                               ['neck', 'right_hip'], ['right_hip', 'right_knee'], ['right_knee', 'right_ankle'],
                               ['neck', 'left_hip'], ['left_hip', 'left_knee'], ['left_knee', 'left_ankle'],
                               ['neck', 'nose'],
                               ['nose', 'right_eye'], ['right_eye', 'right_ear'],
                               ['nose', 'left_eye'], ['left_eye', 'left_ear']]

        self.num_landmarks = len(self.landmark_names)
        print('self.num_landmarks =', self.num_landmarks)
        print('len(self.landmark_colors =', len(self.landmark_colors))

        
    def get_landmark_names(self):
        return self.landmark_names
    
    def get_landmark_colors(self):
        return self.landmark_colors

    def get_landmark_color_dict(self):
        return self.landmark_color_dict
         
    def apply_to_image(self, rgb_image, draw_output=False, landmarks_to_detect=None):
        if landmarks_to_detect is None:
            landmarks_to_detect = self.default_landmarks_to_detect

        if draw_output:
            output_image = rgb_image.copy()
        else:
            output_image = None

        orig_h, orig_w, orig_c = rgb_image.shape
        orig_ratio = orig_h / orig_w
        target_h = 256
        target_w = 456
        target_ratio = target_h / target_w
        if orig_ratio > target_ratio:
            new_h = target_h
            scale = new_h / orig_h
            new_w = int(round(orig_w * scale))
        else:
            new_w = target_w
            scale = new_w / orig_w
            new_h = int(round(orig_h * scale))
        scaled = cv2.resize(rgb_image, (new_w, new_h))

        if new_h < target_h:
            new_h_offset = int(round((target_h - new_h) / 2.0))
            new_w_offset = 0
        if new_w < target_w:
            new_w_offset = int(round((target_w - new_w) / 2.0))
            new_h_offset = 0

        zero_padded = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        zero_padded[new_h_offset:new_h_offset+new_h, new_w_offset:new_w_offset+new_w, :] = scaled

        skeleton_image_blob = cv2.dnn.blobFromImage(zero_padded, size = (target_w, target_h))

        skeleton_image_blob = cv2.dnn.blobFromImage(zero_padded,
                                                    size = (target_w, target_h),
                                                    mean = (128.0, 128.0, 128.0),
                                                    swapRB = False,
                                                    crop = False,
                                                    ddepth = cv2.CV_32F)

        self.skeleton_model.setInput(skeleton_image_blob)

        skeleton_out = self.skeleton_model.forward(['Mconv7_stage2_L2'])
        confidence_maps = skeleton_out[0][0]
        background_confidence_map = confidence_maps[self.num_landmarks]
        c_map_h, c_map_w = background_confidence_map.shape
        enlarge = 8.0 / scale
        new_c_map_h = int(round(enlarge * c_map_h))
        new_c_map_w = int(round(enlarge * c_map_w))
        scaled_background_confidence_map = cv2.resize(background_confidence_map, (new_c_map_w, new_c_map_h))

        bodies = []
        landmark_dict = {}

        for name in landmarks_to_detect:
            landmark_index = self.landmark_names.index(name)
            confidence_map = confidence_maps[landmark_index]

            scaled_confidence_map = cv2.resize(confidence_map, (new_c_map_w, new_c_map_h))
            min_val, max_val, (min_x, min_y), (max_x, max_y) = cv2.minMaxLoc(scaled_confidence_map)
            background_confidence = scaled_background_confidence_map[max_y, max_x]
            if max_val > background_confidence: 
                landmark_x = int(round((max_x - (new_w_offset/scale))))
                landmark_x = min(orig_w - 1, landmark_x)
                landmark_y = int(round((max_y - (new_h_offset/scale))))
                landmark_y = min(orig_h - 1, landmark_y)
                landmark_dict[name] = (landmark_x, landmark_y)
        
        bodies.append({'box':None, 'ypr':None, 'landmarks':landmark_dict})

        if draw_output:
            print('bodies =', bodies)
            for body in bodies:
                print('body =', body)
                self.draw_skeleton(output_image, body)
        
        return bodies, output_image

        
    def draw_skeleton(self, image, body):
        landmark_dict = body['landmarks']
        for name, xy in landmark_dict.items():
            landmark_x, landmark_y = xy
            radius = 5
            thickness = 2
            color = [0, 0, 255]
            cv2.circle(image, (landmark_x, landmark_y), radius, color, thickness)
            font_scale = 1.0 #2.0
            line_color = [255, 0, 0]
            line_width = 1 #2
            font = cv2.FONT_HERSHEY_PLAIN 
            cv2.putText(image, '{0}'.format(name), (landmark_x, landmark_y), font, font_scale, line_color, line_width, cv2.LINE_AA)
                
            
class HandoffPositionDetector(BodyLandmarkDetector):
    def __init__(self, models_directory, use_neural_compute_stick=False):
        self.landmarks_to_detect = ['neck', 'nose', 'right_shoulder', 'left_shoulder']
        BodyLandmarkDetector.__init__(self, models_directory,
                                      landmarks_to_detect=self.landmarks_to_detect,
                                      use_neural_compute_stick=use_neural_compute_stick)
           
    def apply_to_image(self, rgb_image, draw_output=False):
        if draw_output:
            output_image = rgb_image.copy()
        else:
            output_image = None

        bodies, unused = BodyLandmarkDetector.apply_to_image(self, rgb_image)
        box_scale = 0.3
        new_bodies = []
        for b in bodies:
            landmarks = b['landmarks']
            neck_xy = landmarks.get('neck')
            nose_xy = landmarks.get('nose')
            right_shoulder_xy = landmarks.get('right_shoulder')
            left_shoulder_xy = landmarks.get('left_shoulder')
            front = True
            box = None
            box_width_pix = None
            if neck_xy is not None:
                neck_xy = np.array(neck_xy)
                if right_shoulder_xy is not None:
                    right_shoulder_xy = np.array(right_shoulder_xy)
                    box_width_pix = 2.0 * (box_scale * np.linalg.norm(right_shoulder_xy - neck_xy))
                    if nose_xy is not None: 
                        nose_xy = np.array(nose_xy)
                        neck_to_nose = nose_xy - neck_xy
                        neck_to_right_shoulder = right_shoulder_xy - neck_xy
                        cross_sign = np.sign(np.cross(neck_to_right_shoulder, neck_to_nose))
                        if cross_sign > 0:
                            front = True
                        else:
                            front = False
                    
                if left_shoulder_xy is not None:
                    left_shoulder_xy = np.array(left_shoulder_xy)
                    left_box_width_pix = 2.0 * (box_scale * np.linalg.norm(left_shoulder_xy - neck_xy))
                    if box_width_pix is not None: 
                        box_width_pix = (left_box_width_pix + box_width_pix) / 2.0
                    else:
                        box_width_pix = left_box_width_pix
                    if nose_xy is not None: 
                        nose_xy = np.array(nose_xy)
                        neck_to_nose = nose_xy - neck_xy
                        neck_to_left_shoulder = left_shoulder_xy - neck_xy
                        cross_sign = np.sign(np.cross(neck_to_left_shoulder, neck_to_nose))
                        if cross_sign > 0:
                            front = False
                        else:
                            front = True

                    
            if box_width_pix is not None:
                x_min = neck_xy[0] - (box_width_pix/2.0)
                x_max = neck_xy[0] + (box_width_pix/2.0)
                y_min = neck_xy[1] - (box_width_pix/2.0)
                y_max = neck_xy[1] + (box_width_pix/2.0)
                box = (x_min, y_min, x_max, y_max)
            new_bodies.append({'box':box, 'ypr':None, 'landmarks':landmarks, 'front':front})

        
        if draw_output:
            print('bodies =', bodies)
            for body in new_bodies:
                print('body =', body)
                self.draw_skeleton(output_image, body)
        
        return new_bodies, output_image
