#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import deep_models_shared as dm

class HeadPoseEstimator:
    def __init__(self, models_directory, use_neural_compute_stick=False):
        # Load the models to run on CPU
        #########################################################################
        
        # Specify model directory and load caffe / prototxt models 
        # Load weights and config files from this directory 
        # Note: These models will not run on compute stick
        #########################################################################
        models_dir = models_directory
        print('Using the following directory to load object detector models:', models_dir)
        
        # file with network architecture and other information
        head_detection_model_prototxt_filename = models_dir + '/head_detection/deploy.prototxt'
        # file with network weights
        head_detection_model_caffemodel_filename = models_dir + '/head_detection/res10_300x300_ssd_iter_140000.caffemodel'
        self.face_confidence_threshold = 0.2

        print('attempting to load neural network from files')
        print('prototxt file =', head_detection_model_prototxt_filename)
        print('caffemodel file =', head_detection_model_caffemodel_filename)
        self.head_detection_model = cv2.dnn.readNetFromCaffe(head_detection_model_prototxt_filename, head_detection_model_caffemodel_filename)
        dm.print_model_info(self.head_detection_model, 'head_detection_model')
        
        # Load the models to run on VPU (NCS 2)

        #########################################################################
        # If neural compute stick is available then run the models on Myriad using the command setPreferableTarget
        # Load model directory from the OpenVINO model zoo 
        # Load weights and config files from this directory 
        #
        #########################################################################
        if use_neural_compute_stick:
            print('HeadPoseEstimator.__init__: Attempting to use an Intel Neural Compute Stick 2 using the following command: self.head_detection_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)')
            self.head_detection_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

        head_pose_model_dir = models_dir + '/open_model_zoo/head-pose-estimation-adas-0001/FP32/'
        head_pose_weights_filename = head_pose_model_dir + 'head-pose-estimation-adas-0001.bin'
        head_pose_config_filename = head_pose_model_dir + 'head-pose-estimation-adas-0001.xml'
        self.head_pose_model = cv2.dnn.readNet(head_pose_weights_filename, head_pose_config_filename)
        
        if use_neural_compute_stick:
            print('Not attempting to use a Intel Neural Compute Stick 2 for head pose estimation due to potential errors.')

        dm.print_model_info(self.head_pose_model, 'head_pose_model')

        landmarks_model_dir = models_dir + '/open_model_zoo/facial-landmarks-35-adas-0002/FP32/'
        landmarks_weights_filename = landmarks_model_dir + 'facial-landmarks-35-adas-0002.bin'
        landmarks_config_filename = landmarks_model_dir + 'facial-landmarks-35-adas-0002.xml'
        self.landmarks_model = cv2.dnn.readNet(landmarks_weights_filename, landmarks_config_filename)

        if use_neural_compute_stick:
            print('Not attempting to use a Intel Neural Compute Stick 2 for facial landmarks due to potential errors.')

        ### The rest of the code is as is
        dm.print_model_info(self.head_pose_model, 'head_pose_model')

        
        dm.print_model_info(self.landmarks_model, 'landmarks_model')
        
        self.landmark_names = ['right_eye_left', 'right_eye_right',
                               'left_eye_right', 'left_eye_left', 'nose_tip',
                               'nose_bottom', 'nose_right', 'nose_left', 'mouth_right',
                               'mouth_left', 'mouth_top', 'mouth_bottom',
                               'right_eyebrow_right', 'right_eyebrow_middle', 'right_eyebrow_left',
                               'left_eyebrow_right', 'left_eyebrow_middle', 'left_eyebrow_left',
                               'right_cheek_18', 'right_cheek_19', 'right_cheek_20', 'right_cheek_21',
                               'right_cheek_22', 'right_cheek_23', 'right_cheek_24',
                               'chin_right', 'chin_middle', 'chin_left',
                               'left_cheek_28', 'left_cheek_29', 'left_cheek_30', 'left_cheek_31',
                               'left_cheek_32', 'left_cheek_33', 'left_cheek_34']
    

    def get_landmark_names(self):
        return self.landmark_names

    def get_landmark_colors(self):
        return None

    def get_landmark_color_dict(self):
        return None

    def detect_faces(self, rgb_image):
        orig_h, orig_w, c = rgb_image.shape
        face_image = rgb_image
        rot_h, rot_w, c = face_image.shape
        # Assumes that the width is smaller than the height, and crop
        # a width x width square image from the top.
        square_face_image = face_image[:rot_w, :, :]
        sqr_h, sqr_w, c = square_face_image.shape
        network_image = cv2.resize(square_face_image, (300, 300))
        # Some magic numbers came from
        # https://www.pyimagesearch.com/2018/02/26/face-detection-with-opencv-and-deep-learning/
        face_image_blob = cv2.dnn.blobFromImage(network_image, 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.head_detection_model.setInput(face_image_blob)
        face_detections = self.head_detection_model.forward()[0,0,:,:]
        confidence_mask = face_detections[:, 2] > self.face_confidence_threshold
        face_detections = face_detections[confidence_mask]
        coordinates = face_detections[:, 3:7]
        # Scale and rotate coordinates to the original image
        coordinates = coordinates * np.array([sqr_w, sqr_h, sqr_w, sqr_h])

        face_id = 0
        boxes = []

        for x0, y0, x1, y1 in coordinates:
            orig_y0 = y0
            orig_y1 = y1
            orig_x0 = x0
            orig_x1 = x1
            face_id += 1
            bounding_box = [orig_x0, orig_y0, orig_x1, orig_y1]
            boxes.append(bounding_box)

        return boxes

    
    def get_sub_image(self, rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15):
        if enlarge_box:
            scale = enlarge_scale
            orig_h, orig_w, c = rgb_image.shape

            x0 = bounding_box[0]
            y0 = bounding_box[1]
            x1 = bounding_box[2]
            y1 = bounding_box[3]

            m_x = (x1 + x0) / 2.0
            m_y = (y1 + y0) / 2.0

            b_w = x1 - x0
            b_h = y1 - y0

            b_w = scale * b_w
            b_h = scale * b_h

            x0 = int(round(m_x - (b_w/2.0)))
            x1 = int(round(m_x + (b_w/2.0)))
            y0 = int(round(m_y - (b_h/2.0)))
            y1 = int(round(m_y + (b_h/2.0)))

            x0 = max(0, x0)
            x1 = min(orig_w, x1)
            y0 = max(0, y0)
            y1 = min(orig_h, y1)
        else: 
            x0 = int(round(bounding_box[0]))
            y0 = int(round(bounding_box[1]))
            x1 = int(round(bounding_box[2]))
            y1 = int(round(bounding_box[3]))

        actual_bounding_box = [x0, y0, x1, y1]
        image_to_crop = rgb_image
        sub_image = image_to_crop[y0:y1, x0:x1, :]
        return sub_image, actual_bounding_box
    

    def estimate_head_pose(self, rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15):
        face_crop_image, actual_bounding_box = self.get_sub_image(rgb_image, bounding_box, enlarge_box=enlarge_box, enlarge_scale=enlarge_scale)
        sqr_h, sqr_w, c = face_crop_image.shape

        if (sqr_h > 0) and (sqr_w > 0):
            head_pose_image_blob = cv2.dnn.blobFromImage(face_crop_image,
                                                         size=(60, 60),
                                                         swapRB=False,
                                                         crop=False,
                                                         ddepth=cv2.CV_32F)
            self.head_pose_model.setInput(head_pose_image_blob)
            head_pose_out = self.head_pose_model.forward(['angle_r_fc', 'angle_p_fc', 'angle_y_fc'])
            rpy = head_pose_out
            roll = rpy[0][0][0]
            pitch = rpy[1][0][0]
            yaw = rpy[2][0][0]
            pitch = pitch * np.pi/180.0
            roll = roll * np.pi/180.0
            yaw = yaw * np.pi/180.0
            
            return yaw, pitch, roll

        return None, None, None


    def detect_facial_landmarks(self, rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15):
        face_crop_image, actual_bounding_box = self.get_sub_image(rgb_image, bounding_box, enlarge_box=enlarge_box, enlarge_scale=enlarge_scale)
        sqr_h, sqr_w, c = face_crop_image.shape

        if (sqr_h > 0) and (sqr_w > 0):
            landmarks_image_blob = cv2.dnn.blobFromImage(face_crop_image,
                                                         size=(60, 60),
                                                         swapRB=False,
                                                         crop=False,
                                                         ddepth=cv2.CV_32F)
            self.landmarks_model.setInput(landmarks_image_blob)
            landmarks_out = self.landmarks_model.forward()

            s = landmarks_out.shape
            out = np.reshape(landmarks_out[0], (s[1]//2, 2))
            x0, y0, x1, y1 = actual_bounding_box

            landmarks = {}
            for n, v in enumerate(out):
                x = int(round((v[0] * sqr_w) + x0))
                y = int(round((v[1] * sqr_h) + y0))
                name = self.landmark_names[n]
                landmarks[name] = (x,y)

            return landmarks, self.landmark_names.copy()
        return None, None


    def draw_bounding_box(self, image, bounding_box):
        x0 = int(round(bounding_box[0]))
        y0 = int(round(bounding_box[1]))
        x1 = int(round(bounding_box[2]))
        y1 = int(round(bounding_box[3]))
        color = (0, 0, 255)
        thickness = 2
        cv2.rectangle(image, (x0, y0), (x1, y1), color, thickness)

        
    def draw_head_pose(self, image, yaw, pitch, roll, bounding_box):
        x0, y0, x1, y1 = bounding_box
        face_x = (x1 + x0) / 2.0
        face_y = (y1 + y0) / 2.0
        #
        # opencv uses right-handed coordinate system
        # x points to the right of the image
        # y points to the bottom of the image
        # z points into the image
        #

        h, w, c = image.shape
        camera_center = (w/2.0, h/2.0)
        #For rendering with an unknown camera
        focal_length = 50.0 
        camera_matrix = np.array([[focal_length, 0.0,          camera_center[0]],
                                  [0.0,          focal_length, camera_center[1]],
                                  [0.0,          0.0,          1.0]])
        face_translation = np.array([0.0, 0.0, 3000.0])
        distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0])
        # negate the directions of the y and z axes
        axes = np.array([[2000.0,  0.0,      0.0   ],
                         [0.0,     -2000.0,  0.0   ],
                         [0.0,     0.0,      -2000.0],
                         [0.0,     0.0,      0.0   ]])
        head_ypr = np.array([-yaw, pitch, roll])
        rotation_mat = Rotation.from_euler('yxz', head_ypr).as_matrix()
        rotation_vec, jacobian = cv2.Rodrigues(rotation_mat)
        image_points, jacobian = cv2.projectPoints(axes, rotation_vec, face_translation, camera_matrix, distortion_coefficients)
        face_pix = np.array([face_x, face_y])

        origin = image_points[3].ravel()
        x_axis = (image_points[0].ravel() - origin) + face_pix
        y_axis = (image_points[1].ravel() - origin) + face_pix
        z_axis = (image_points[2].ravel() - origin) + face_pix

        p0 = tuple(np.int32(np.round(face_pix)))
        p1 = tuple(np.int32(np.round(x_axis)))
        cv2.line(image, p0, p1, (0, 0, 255), 2)
        p1 = tuple(np.int32(np.round(y_axis)))
        cv2.line(image, p0, p1, (0, 255, 0), 2)
        p1 = tuple(np.int32(np.round(z_axis)))
        cv2.line(image, p0, p1, (255, 0, 0), 2)

        
    def draw_landmarks(self, image, landmarks):
        for name, xy in landmarks.items():
            x = xy[0]
            y = xy[1]
            if 'mouth' in name:
                color = (255, 0, 0)
            elif 'nose' in name:
                color = (0, 255, 0)
            elif 'eyebrow' in name:
                color = (0, 0, 0)
            elif 'right_eye' in name:
                color = (255, 255, 0)
            elif 'left_eye' in name:
                color = (0, 255, 255)
            elif 'chin' in name:
                color = (255, 0, 255)
            else:
                color = (0, 0, 255)
            cv2.circle(image, (x,y), 2, color, 1)
            font_scale = 1.0
            line_color = [0, 0, 0]
            line_width = 1
            font = cv2.FONT_HERSHEY_PLAIN 

            
    def apply_to_image(self, rgb_image, draw_output=False):
        if draw_output:
            output_image = rgb_image.copy()
        else:
            output_image = None

        heads = []
        boxes = self.detect_faces(rgb_image)
        facial_landmark_names = self.landmark_names.copy()
        for bounding_box in boxes:
            if draw_output: 
                self.draw_bounding_box(output_image, bounding_box)
            yaw, pitch, roll = self.estimate_head_pose(rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15)
            if yaw is not None: 
                ypr = (yaw, pitch, roll)
                if draw_output: 
                    self.draw_head_pose(output_image, yaw, pitch, roll, bounding_box)
            else:
                ypr = None
            landmarks, landmark_names = self.detect_facial_landmarks(rgb_image, bounding_box, enlarge_box=True, enlarge_scale=1.15)
            if (landmarks is not None) and draw_output: 
                self.draw_landmarks(output_image, landmarks)
            heads.append({'box':bounding_box, 'ypr':ypr, 'landmarks':landmarks})

        return heads, output_image
