#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from scipy.spatial.transform import Rotation
import hello_helpers.hello_ros_viz as hr
from numba_image_to_pointcloud import numba_image_to_pointcloud
import hello_helpers.fit_plane as fp


def filter_points(points_array, camera_matrix, box_2d, min_box_side_m, max_box_side_m): 
    # Decompose the camera matrix.
    f_x = camera_matrix[0,0]
    c_x = camera_matrix[0,2]
    f_y = camera_matrix[1,1]
    c_y = camera_matrix[1,2]
    
    # These need to be flipped with respect to the basic update
    # function to account for the rotation applied as part of the
    # head orientation estimation.
    x0, y0, x1, y1 = box_2d
    detection_box_width_pix = y1 - y0
    detection_box_height_pix = x1 - x0

    z_min = min_box_side_m * min(f_x/detection_box_width_pix, f_y/detection_box_height_pix)
    z_max = max_box_side_m * max(f_x/detection_box_width_pix, f_y/detection_box_height_pix)

    z = points_array[:,2]
    mask_z = (z > z_min) & (z < z_max)

    # TODO: Handle situations when the cropped rectangle contains no
    # reasonable depth values.

    # Second, filter for depths that are within one maximum head
    # length away from the median depth.
    remaining_z = z[mask_z]
    out_points = np.empty((0,3), dtype=np.float32)
    if len(remaining_z) > 0:
        median_z = np.median(remaining_z)
        min_z = median_z - max_box_side_m
        max_z = median_z + max_box_side_m
        mask_z = (z > min_z) & (z < max_z)
        remaining_z = z[mask_z]
        if len(remaining_z) > 0: 
            out_points = points_array[mask_z]

    return out_points

            
def landmarks_2d_to_3d(landmarks, camera_matrix, depth_image, default_z_3d):

    f_x = camera_matrix[0,0]
    c_x = camera_matrix[0,2]
    f_y = camera_matrix[1,1]
    c_y = camera_matrix[1,2]

    landmarks_3d = {}
    for name, xy in landmarks.items():
        x, y = xy
        z = depth_image[y,x]
        if z > 0: 
            z_3d = z / 1000.0
        else:
            z_3d = default_z_3d
        x_3d = ((x - c_x) / f_x) * z_3d
        y_3d = ((y - c_y) / f_y) * z_3d
        landmarks_3d[name] = (x_3d, y_3d, z_3d)

    return landmarks_3d


def bounding_box_2d_to_3d(points_array, box_2d, camera_matrix, head_to_camera_mat=None, fit_plane=False):

    x0, y0, x1, y1 = box_2d

    f_x = camera_matrix[0,0]
    c_x = camera_matrix[0,2]
    f_y = camera_matrix[1,1]
    c_y = camera_matrix[1,2]

    center_xy_pix = np.array([0.0, 0.0])
    center_xy_pix[0] = (x0 + x1)/2.0
    center_xy_pix[1] = (y0 + y1)/2.0
    # These need to be flipped with respect to the basic update
    # function to account for the rotation applied as part of the
    # head orientation estimation.
    detection_box_width_pix = y1 - y0
    detection_box_height_pix = x1 - x0

    num_points = points_array.shape[0]
    if num_points >= 1: 
        box_depth = np.median(points_array, axis=0)[2]
    else:
        print('WARNING: No reasonable depth image points available in the detected rectangle. No work around currently implemented for lack of depth estimate.')
        return None

    # Convert to 3D point in meters using the camera matrix.
    center_z = box_depth
    center_x = ((center_xy_pix[0] - c_x) / f_x) * center_z
    center_y = ((center_xy_pix[1] - c_y) / f_y) * center_z

    detection_box_width_m = (detection_box_width_pix / f_x) * box_depth
    detection_box_height_m = (detection_box_height_pix / f_y) * box_depth

    if head_to_camera_mat is None: 
        R = np.identity(3)
        quaternion = Rotation.from_matrix(R).as_quat()
        x_axis = R[:3,0]
        y_axis = R[:3,1]
        z_axis = R[:3,2]
    else: 
        quaternion = Rotation.from_matrix(head_to_camera_mat).as_quat()
        x_axis = head_to_camera_mat[:3,0]
        y_axis = head_to_camera_mat[:3,1]
        z_axis = head_to_camera_mat[:3,2]

    plane = None

    # Find suitable 3D points within the Face detection box. If there
    # are too few points, do not proceed with fitting a plane.
    num_points = points_array.shape[0]
    min_number_of_points_for_plane_fitting = 16
    enough_points = (num_points >= min_number_of_points_for_plane_fitting)
    if fit_plane and (not enough_points):
        print('WARNING: There are too few points from the depth image for plane fitting. number of points =', num_points)
    elif fit_plane:
        plane = fp.FitPlane()
        plane.fit_svd(points_array, verbose=False)

        #####################################
        # Find the points on the fit plane corresponding with the
        # four Face rectangle corners. Then, use the mean of the 4
        # points as the 3D center for the marker.
        d = plane.d
        n = plane.n

        def pix_to_plane(pix_x, pix_y):
            z = 1.0
            x = ((pix_x - c_x) / f_x) * z
            y = ((pix_y - c_y) / f_y) * z
            point = np.array([x, y, z])
            ray = point/np.linalg.norm(point)
            point = ((d / np.matmul(n.transpose(), ray)) * ray).flatten()
            return point

        corners = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
        corner_points = []
        total_corner = np.array([0.0, 0.0, 0.0])
        for (pix_x, pix_y) in corners:
            corner_point = pix_to_plane(pix_x, pix_y)
            total_corner += corner_point
            corner_points.append(corner_point)
        center_x, center_y, center_z = total_corner / 4.0

        # Use the corners on the fit plane to estimate the x and y
        # axes for the marker.
        top_left, top_right, bottom_right, bottom_left = corner_points

        y_axis = (top_left + top_right) - (bottom_left + bottom_right)
        y_length = np.linalg.norm(y_axis)
        if y_length > 0.0: 
            y_axis = y_axis/y_length
        else:
            y_axis = None

        x_axis = (top_right + bottom_right) - (top_left + bottom_left)
        x_length = np.linalg.norm(x_axis)
        if x_length > 0.0:
            x_axis = x_axis/x_length
        else:
            x_axis = None

        #####################################

        plane_normal = plane.get_plane_normal()

        if x_axis is not None: 
            old_x_axis = np.reshape(x_axis, (3,1))
        else:
            old_x_axis = np.array([1.0, 0.0, 0.0])

        if y_axis is not None: 
            old_y_axis = np.reshape(y_axis, (3,1))
        else:
            old_y_axis = np.array([0.0, 1.0, 0.0])

        new_z_axis = plane_normal
        # The following methods directly use the z axis from the
        # plane fit.
        if (x_axis is not None) and (y_axis is None):
            # For tests with the two wrist markers, this method
            # appeared to be the best. It showed the most
            # stability. In particular, it showed the least
            # rotation around the normal to the marker.
            new_x_axis = old_x_axis - (np.matmul(new_z_axis.transpose(), old_x_axis) * new_z_axis)
            new_x_axis = new_x_axis/np.linalg.norm(new_x_axis)
            new_y_axis = np.reshape(np.cross(new_z_axis.flatten(), new_x_axis.flatten()), (3,1))
        elif (x_axis is None) and (y_axis is not None):
            new_y_axis = old_y_axis - (np.matmul(new_z_axis.transpose(), old_y_axis) * new_z_axis)
            new_y_axis = new_y_axis/np.linalg.norm(new_y_axis)
            new_x_axis = np.reshape(np.cross(new_y_axis.flatten(), new_z_axis.flatten()), (3,1))
        elif False:
            # Attempt to reduce bias due to selecting one of the
            # old axes by averaging the results from both axes.
            new_x_axis_1 = old_x_axis - (np.matmul(new_z_axis.transpose(), old_x_axis) * new_z_axis)
            new_x_axis_1 = new_x_axis_1/np.linalg.norm(new_x_axis_1)

            new_y_axis_1 = np.reshape(np.cross(new_z_axis.flatten(), new_x_axis_1.flatten()), (3,1))

            new_y_axis_2 = old_y_axis - (np.matmul(new_z_axis.transpose(), old_y_axis) * new_z_axis)
            new_y_axis_2 = new_y_axis_2/np.linalg.norm(new_y_axis_2)

            new_y_axis = (new_y_axis_1 + new_y_axis_2)/2.0
            new_y_axis = new_y_axis/np.linalg.norm(new_y_axis)
            new_x_axis = np.reshape(np.cross(new_y_axis.flatten(), new_z_axis.flatten()), (3,1))
        else:
            if (x_axis is None) and (y_axis is None):
                print('WARNING: The detected corners did not project to reasonable 3D points on the fit plane.')
                #print('         corners[0] =', corners[0])
                new_y_axis = old_y_axis
                new_x_axis = old_x_axis
            else: 
                # Attempt to reduce bias due to selecting one of the
                # old axes by averaging the results from both axes.
                new_y_axis_1 = old_y_axis - (np.matmul(new_z_axis.transpose(), old_y_axis) * new_z_axis)
                new_y_axis_1 = new_y_axis_1/np.linalg.norm(new_y_axis_1)

                new_x_axis_1 = np.reshape(np.cross(new_y_axis_1.flatten(), new_z_axis.flatten()), (3,1))

                new_x_axis_2 = old_x_axis - (np.matmul(new_z_axis.transpose(), old_x_axis) * new_z_axis)
                new_x_axis_2 = new_x_axis_2/np.linalg.norm(new_x_axis_2)

                new_x_axis = (new_x_axis_1 + new_x_axis_2)/2.0
                new_x_axis = new_x_axis/np.linalg.norm(new_x_axis)
                new_y_axis = np.reshape(np.cross(new_z_axis.flatten(), new_x_axis.flatten()), (3,1))

        x_axis = new_x_axis.flatten()
        y_axis = new_y_axis.flatten()
        z_axis = new_z_axis.flatten()

        R = np.identity(3)
        R[:3,0] = x_axis
        R[:3,1] = y_axis
        R[:3,2] = z_axis

        quaternion = Rotation.from_matrix(R).as_quat()

    if plane is not None:
        simple_plane = {'n': plane.n, 'd': plane.d}
    else:
        simple_plane = None
        
    box_3d = {'center_xyz': (center_x, center_y, center_z),
              'quaternion': quaternion,
              'x_axis': x_axis, 'y_axis': y_axis, 'z_axis': z_axis,
              'width_m': detection_box_width_m,
              'height_m': detection_box_height_m,
              'width_pix': detection_box_width_pix,
              'height_pix': detection_box_height_pix,
              'plane': simple_plane}

    return box_3d


def detections_2d_to_3d(detections_2d, rgb_image, camera_info, depth_image, fit_plane=False, min_box_side_m=None, max_box_side_m=None):

    orig_h, orig_w, c = rgb_image.shape

    def clip_xy(x_in, y_in):
        x_out = x_in
        y_out = y_in
        x_out = max(0, x_out)
        x_out = min(orig_w - 1, x_out)
        y_out = max(0, y_out)
        y_out = min(orig_h - 1, y_out)
        return x_out, y_out
        
    camera_matrix = np.reshape(camera_info.K, (3,3))
    distortion_coefficients = np.array(camera_info.D)
    
    def clockwise_rotate_bounding_box(box_2d): 
        x0, y0, x1, y1 = box_2d
        orig_x0 = (orig_w - 1) - y1
        orig_y0 = x0
        orig_x1 = (orig_w - 1) - y0
        orig_y1 = x1
        return (orig_x0, orig_y0, orig_x1, orig_y1)
    
    def counterclockwise_rotate_bounding_box(box_2d): 
        x0, y0, x1, y1 = box_2d
        orig_x0 = y0
        orig_y0 = (orig_h - 1) - x1
        orig_x1 = y1
        orig_y1 = (orig_h - 1) - x0
        return (orig_x0, orig_y0, orig_x1, orig_y1)
    
    def clockwise_rotate_xy(x, y):
        return ((orig_w - 1) - y), x

    def counterclockwise_rotate_xy(x, y):
        return y, (orig_h - 1) - x

    rotvec = np.array([0.0, 0.0, 1.0]) * (-np.pi/2.0)
    counterclockwise_rotate_mat = Rotation.from_rotvec(rotvec).as_matrix()

    detections_3d = []
    
    for h in detections_2d:
        box_3d = None
        landmarks_3d = None
        box_2d = h.get('box')
        label = h.get('label')
        ypr = h.get('ypr')
        landmarks_2d = h.get('landmarks')
        points_3d = None
        front = h.get('front')
        
        if box_2d is not None: 
            box_2d = counterclockwise_rotate_bounding_box(box_2d)
            x0, y0, x1, y1 = box_2d
            x0, y0 = clip_xy(x0, y0)
            x1, y1 = clip_xy(x1, y1)
            
            if ((x0 < 0) or (y0 < 0) or (x1 < 0) or (y1 < 0) or
                (x0 >= orig_w) or (y0 >= orig_h) or (x1 >= orig_w) or (y1 >= orig_h) or
                (x0 >= x1) or (y0 >= y1)):
                print('---------------')
                print('WARNING: detection bounding box goes outside of the original image dimensions or has other issues, so ignoring detection.')
                print('box_2d =', box_2d)
                print('rgb_image.shape =', rgb_image.shape)
                print('---------------')
                box_2d = None

        if landmarks_2d is not None:
            rotated_landmarks_2d = {}
            for name, xy in landmarks_2d.items():
                rotated_xy = counterclockwise_rotate_xy(xy[0], xy[1])
                x0, y0 = rotated_xy
                x0, y0 = clip_xy(x0, y0)
                rotated_landmarks_2d[name] = (x0, y0)
            landmarks_2d = rotated_landmarks_2d

        if ypr is not None: 
            yaw, pitch, roll = ypr
            head_ypr = np.array([-yaw, pitch, roll])
            rotation_mat = Rotation.from_euler('yxz', head_ypr).as_matrix()
            head_to_camera_mat = np.matmul(counterclockwise_rotate_mat, rotation_mat)
        else:
            head_to_camera_mat = counterclockwise_rotate_mat

        if (box_2d is not None) or (landmarks_2d is not None) or (ypr is not None):

            box_depth_m = 0.0
            if box_2d is not None:
                points_3d = numba_image_to_pointcloud(depth_image, box_2d, camera_matrix)                
                if (min_box_side_m is not None) and (max_box_side_m is not None):
                    points_3d = filter_points(points_3d, camera_matrix, box_2d, min_box_side_m, max_box_side_m)
                box_3d = bounding_box_2d_to_3d(points_3d, box_2d, camera_matrix, head_to_camera_mat=head_to_camera_mat, fit_plane=fit_plane)
                if box_3d is None:
                    box_depth_m = None
                else:
                    box_depth_m = box_3d['center_xyz'][2]
                
            if landmarks_2d is not None:
                if box_depth_m is None:
                    landmarks_3d = None
                else: 
                    landmarks_3d = landmarks_2d_to_3d(landmarks_2d, camera_matrix, depth_image, box_depth_m)

        detections_3d.append({'box_3d':box_3d,
                              'landmarks_3d':landmarks_3d,
                              'box_2d':box_2d,
                              'label':label,
                              'ypr':ypr,
                              'landmarks_2d':landmarks_2d,
                              'points_3d':points_3d,
                              'front':front})

    return detections_3d
