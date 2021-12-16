#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import math

import yaml
import gzip

import struct
import threading
from collections import deque

import stretch_funmap.numba_height_image as nh
from stretch_funmap.numba_create_plane_image import numba_create_plane_image, numba_correct_height_image, transform_original_to_corrected, transform_corrected_to_original

from scipy.spatial.transform import Rotation

from copy import deepcopy

class Colormap:
    def __init__(self, colormap=cv2.COLORMAP_HSV):
        self.colormap = colormap
        gray_image = np.uint8(np.reshape(np.arange(256), (256, 1)))
        gray_to_bgr = cv2.applyColorMap(gray_image, self.colormap)
        gray_to_bgr = np.reshape(gray_to_bgr, (256, 3))
        rgb = np.zeros((256,3), np.uint8)
        rgb[:,0] = gray_to_bgr[:,2]
        rgb[:,1] = gray_to_bgr[:,1]
        rgb[:,2] = gray_to_bgr[:,0]
        self.gray_to_color = rgb

    def get_color(self, gray_value):
        index = int(round(gray_value)) % 256
        return self.gray_to_color[index]

    def get_map_array(self):
        return self.gray_to_color
        
    
class VolumeOfInterest:
    def __init__(self, frame_id, origin, axes, x_in_m, y_in_m, z_in_m):
        ########################################################
        # frame_id : frame with respect to which the volume of
        # interest (VOI) is defined
        #
        # origin : x_f, y_f, z_f with respect to frame_id coordinate
        # system, f, that defines the origin of the volume of interest
        # (VOI). The origin is at a corner of the VOI.
        #
        # axes : A 3x3 rotation matrix with columns that define the
        # axes of the volume of interest (VOI) with respect to the
        # frame_id coordinate system. Together with the origin, the
        # axes form a right-handed coordinate system. The VOI occupies
        # the positive octant of the coordinate system. The column
        # vectors of the matrix are unit vectors orthogonal to one
        # another. The x axis of the VOI is column 0, the y axis is
        # column 1, and the z axis is column 2.
        #
        # x_in_m : length in meters of the edges of the volume
        # parallel to the x axis defined by axes.
        #
        # y_in_m : length in meters of the edges of the volume
        # parallel to the y axis defined by axes.
        #
        # z_in_m : length in meters of the edges of the volume
        # parallel to the z axis defined by axes.
        #
        ########################################################
        
        self.frame_id = frame_id
        # origin with respect to frame_id coordinate system
        self.origin = origin
        # axes with respect to frame_id coordinate system
        self.axes = axes
        self.points_in_voi_to_frame_id_mat = np.identity(4)
        self.points_in_voi_to_frame_id_mat[:3,:3] = self.axes
        self.points_in_voi_to_frame_id_mat[:3,3] = self.origin
        self.points_in_frame_id_to_voi_mat = np.linalg.inv(self.points_in_voi_to_frame_id_mat)
        self.x_in_m = x_in_m
        self.y_in_m = y_in_m
        self.z_in_m = z_in_m

    def get_points_to_voi_matrix(self, points_to_frame_id_mat):
        points_to_voi_mat = np.matmul(self.points_in_frame_id_to_voi_mat, points_to_frame_id_mat)
        return points_to_voi_mat

    def change_frame(self, points_in_old_frame_to_new_frame_mat, new_frame_id):
        # Assumes the input matrix defines a rigid body transformation.
        
        # translate the origin
        origin = list(self.origin)
        origin.append(1.0)
        origin = np.array(origin)
        new_origin = np.matmul(points_in_old_frame_to_new_frame_mat, origin)
        # rotate the axes
        new_axes = np.matmul(points_in_old_frame_to_new_frame_mat[:3,:3], self.axes)
        # transform transforms
        new_points_in_voi_to_frame_id_mat = np.matmul(points_in_old_frame_to_new_frame_mat, self.points_in_voi_to_frame_id_mat)
        new_points_in_frame_id_to_voi_mat = np.linalg.inv(new_points_in_voi_to_frame_id_mat)
        # set
        self.frame_id = new_frame_id
        self.origin = new_origin
        self.axes = new_axes
        self.points_in_voi_to_frame_id_mat = new_points_in_voi_to_frame_id_mat
        self.points_in_frame_id_to_voi_mat = new_points_in_frame_id_to_voi_mat
        
        
    def serialize(self):
        # return dictionary with the parameters needed to recreate it
        data = {'frame_id': self.frame_id, 'origin': self.origin, 'axes': self.axes, 'x_in_m': self.x_in_m, 'y_in_m': self.y_in_m, 'z_in_m': self.z_in_m}
        return data

    @classmethod
    def from_serialization(self, data):
        d = data
        voi = VolumeOfInterest(d['frame_id'], np.array(d['origin']), np.array(d['axes']), d['x_in_m'], d['y_in_m'], d['z_in_m'])
        return voi

    
class MaxHeightImage:
    def __init__(self, volume_of_interest, m_per_pix, pixel_dtype, m_per_height_unit=None, use_camera_depth_image=False, image=None, rgb_image=None, camera_depth_image=None):
        # MaxHeightImage creates a 2D image that represents 3D points
        # strictly within a volume of interest (VOI), so excluding
        # points on the border of the VOI. The image can be thought of
        # as sitting at the bottom of the VOI at z=0. Each of the
        # image's pixels represents the point that is highest above
        # the pixel (maximum z value). In other words, the image
        # represents the top surface of the VOI. It is a discretized
        # approximation to a surface function f(x,y) -> z where z =
        # Max(Points(x,y).z). If the pixel has a value of 0, then no
        # points are within the volume above the pixel.
        #
        # volume_of_interest : volume of interest (VOI) that will be
        # represented by the max height image (MHI). The MHI will
        # strictly represent the interior of the VOI and ignore a
        # point sitting exactly on the border of the VOI. The MHI can
        # be thought of as sitting at the bottom of the VOI at z=0.
        #
        # m_per_pix : meters per pixel that defines the spatial
        # resolution of the max height image. Each pixel represents a
        # column with a square cross section within the volume of
        # interest (VOI). m_per_pix is the length of the edges of this
        # square cross section.
        #
        # m_per_height_unit : meters per unit of height is an optional
        # parameter that sets the discretization of the 3D point
        # height. The pixel_type results in a lower bound for
        # m_per_height_unit that depends on the height of the volume
        # of interest. This is due to the pixel type limiting the
        # number of distinct height values a pixel can represent
        # (e.g., 254 with np.uint8). If m_per_height_unit is not
        # specified, the max height image attempts to use square
        # voxels for the discretization by setting m_per_height_unit =
        # m_per_pix. If this is unachievable, it will use the highest
        # resolution it can given the pixel_type and produce a warning
        # message.
        #
        # pixel_dtype : Numpy dtype for the max height image (MHI)
        # pixels, which also defines the maximum resolution in height.
        #
        self.supported_dtypes = [np.uint8, np.uint16, np.uint32, np.uint64, np.float32, np.float64]

        if pixel_dtype not in self.supported_dtypes:
            print('ERROR: Attempt to initialize MaxHeightImage with a pixel_dtype that is not currently supported')
            print('       pixel_dtype =', pixel_dtype)
            print('       self.supported_dtypes =', self.supported_dtypes)
            assert(pixel_dtype in self.supported_dtypes)

        self.m_per_pix = m_per_pix
        
        # Copy the volume of interest to avoid issues of it being
        # mutated after creation of the max height image.
        self.voi = deepcopy(volume_of_interest)

        # Available to hold a correction transform applied to the
        # height image. This transformation will not be performed on
        # the VOI.
        self.transform_original_to_corrected = None
        self.transform_corrected_to_original = None

        # Use ceil so that pixels on the far edges of the image can
        # represent column volumes truncated by the volume of interest
        # borders.
        num_x_bins = np.int(np.ceil(self.voi.x_in_m / self.m_per_pix))
        num_y_bins = np.int(np.ceil(self.voi.y_in_m / self.m_per_pix))

        def find_minimum_m_per_height_unit(z_in_m, max_z_bins):
            m_per_height_unit = z_in_m / (max_z_bins - 1)
            # Check for off by one error and correct it if
            # necessary. This type of error is likely due to float
            # precision, int casting, and ceil issues.
            num_z_bins = 1 + np.int(np.ceil(z_in_m / m_per_height_unit))
            if num_z_bins > max_z_bins:
                m_per_height_unit = z_in_m / (max_z_bins - 2)
            return m_per_height_unit    
        
        if np.issubdtype(pixel_dtype, np.integer):
            max_z_bins = np.iinfo(pixel_dtype).max
            
            if m_per_height_unit is None:
                # Two plausible default behaviors: 1) set to be the
                # same as the x and y resolution 2) set to the maximum
                # resolution achievable given the pixel type

                # For now, I'm going to make the default be the
                # maximum resolution achievable given the pixel type.
                self.m_per_height_unit = find_minimum_m_per_height_unit(self.voi.z_in_m, max_z_bins)
            else:
                self.m_per_height_unit = m_per_height_unit

            # image pixels are integers with limited range

            # Use ceil so that the maximum height voxels can be
            # truncated by the volume of interest borders. Add one to
            # account for 0 representing that no 3D points are in the
            # pixel column volume.
            num_z_bins = 1 + np.int(np.ceil(self.voi.z_in_m / self.m_per_height_unit))
            
            if num_z_bins > max_z_bins:
                print('WARNING: Unable to initialize MaxHeightImage with requested or default height resolution. Instead, using the highest resolution available for the given pixel_dtype. Consider changing pixel_dtype.')
                print('         attempted self.m_per_height_unit =', self.m_per_height_unit)
                print('         attempted num_z_bins =', num_z_bins)
                print('         max_z_bins =', max_z_bins)
                print('         pixel_dtype =', pixel_dtype)
                print('         self.supported_dtypes =', self.supported_dtypes)

                num_z_bins = max_z_bins
                self.m_per_height_unit = find_minimum_m_per_height_unit(self.voi.z_in_m, max_z_bins)
                print('         actual num_z_bins =', num_z_bins)
                print('         actual self.m_per_height_unit =', self.m_per_height_unit)
                
        elif np.issubdtype(pixel_dtype, np.floating):
            if m_per_height_unit is not None:
                print('WARNING: Ignoring provided m_per_height_unit, since pixel_dtype is a float. Float pixels are represented in meters and use no scaling, binning, or discretization.')
                print('         provided m_per_height_unit =', m_per_height_unit)
            self.m_per_height_unit = None

        if image is None: 
            self.image = np.zeros((num_y_bins, num_x_bins), pixel_dtype)
        else:
            # Check that the provided image is consistent with the other parameters
            s = image.shape
            assert(len(s) == 2)
            assert(s[0] == num_y_bins)
            assert(s[1] == num_x_bins)
            assert(image.dtype == pixel_dtype)
            assert(m_per_height_unit == self.m_per_height_unit)
            self.image = image


        if rgb_image is None:
            # The default behavior is not to have an associated RGB image.
            self.rgb_image = None
        else:
            # Check that the provided image is consistent with the other parameters
            s = rgb_image.shape
            assert(len(s) == 3)
            assert(s[0] == num_y_bins)
            assert(s[1] == num_x_bins)
            assert(image.dtype == np.uint8)
            self.rgb_image = rgb_image


        # This conversion is hardcoded in numba code.  4 cm per unit
        # results in 10.16 meter max = 254*0.04.  The D435i is listed
        # as having an approximately 10 meter maximum range.
        # camera_depth = 1 + int(round(z_p/0.04))
        self.camera_depth_m_per_unit = 0.04
        self.camera_depth_min = 1
        
        if use_camera_depth_image:
            if camera_depth_image is None: 
                self.camera_depth_image = np.zeros((num_y_bins, num_x_bins), np.uint8)
            else:
                # Check that the provided camera depth image is consistent with the other parameters
                s = camera_depth_image.shape
                assert(len(s) == 2)
                assert(s[0] == num_y_bins)
                assert(s[1] == num_x_bins)
                assert(image.dtype == np.uint8)
                self.camera_depth_image = camera_depth_image
        else:
            self.camera_depth_image = None
            
        # image_origin : The image origin in 3D space with respect to
        # the volume of interest (VOI) coordinate system. The pixel at
        # image[0,0] is the location of the image origin and is
        # located at the corner of the VOI such that decreasing y,
        # increasing x, and increasing z moves to the interior of the
        # VOI. The pixel at image[0,0] should only represent 3D points
        # that are strictly inside the VOI, not on the border.

        x_v = self.m_per_pix / 0.5
        y_v = self.voi.y_in_m - (self.m_per_pix / 0.5)
        z_v = 0.0
        self.image_origin = np.array([x_v, y_v, z_v])

    def m_to_camera_depth_pix(self, camera_depth_m):
        # This conversion is hardcoded in numba code.
        # 4 cm per unit results in 10.16 meter max = 254*0.04.
        # The D435i is listed as having an approximately 10
        # meter maximum range.
        depth_pix = self.camera_depth_min + int(round(camera_depth_m/self.camera_depth_m_per_unit))
        return depth_pix
        
    def print_info(self):
        print('MaxHeightImage information:')
        print('     image.shape =', self.image.shape)
        print('     image.dtype =', self.image.dtype)
        print('     m_per_pix =', self.m_per_pix)
        print('     m_per_height_unit =', self.m_per_height_unit)
        print('     voi.x_in_m =', self.voi.x_in_m)
        print('     voi.y_in_m =', self.voi.y_in_m)
        print('     voi.z_in_m =', self.voi.z_in_m)

    def clear(self):
        self.image.fill(0)
        if self.rgb_image is not None:
            self.image.fill(0)
        if self.camera_depth_image is not None:
            self.camera_depth_image.fill(0)

    def create_blank_rgb_image(self):
        h, w = self.image.shape
        self.rgb_image = np.zeros((h, w, 3), np.uint8)

    def apply_planar_correction(self, plane_parameters, plane_height_pix):
        # plane_parameters: [alpha, beta, gamma] such that alpha*x + beta*y + gamma = z
        # plane_height_m: The new height for points on the plane in meters
        plane_height_pix = plane_height_pix
        
        self.image, transform_to_corrected = numba_correct_height_image(plane_parameters, self.image, plane_height_pix)
        self.transform_original_to_corrected = transform_to_corrected
        self.transform_corrected_to_original = np.linalg.inv(transform_to_corrected)
            
    def save( self, base_filename, save_visualization=True ):
        print('MaxHeightImage saving to base_filename =', base_filename)

        max_pix = None
        if save_visualization: 
            # Save uint8 png image for visualization purposes. This would
            # be sufficient for uint8 pixel_dtypes, but does not work for
            # uint16.
            if self.image.dtype != np.uint8:
                # Scale the image for better visualization. For example, a
                # float image may not be interpretable in the uint8
                # version without scaling.
                max_pix = np.max(self.image)
                save_image = np.uint8(255.0 * (self.image / max_pix ))
            else:
                save_image = self.image
            visualization_filename = base_filename + '_visualization.png'
            cv2.imwrite(visualization_filename, save_image)
        else:
            visualization_filename = None

        rgb_image_filename = None
        if self.rgb_image is not None:
            rgb_image_filename = base_filename + '_rgb.png'
            cv2.imwrite(rgb_image_filename, self.rgb_image)

        camera_depth_image_filename = None
        if self.camera_depth_image is not None:
            camera_depth_image_filename = base_filename + '_camera_depth.png'
            cv2.imwrite(camera_depth_image_filename, self.camera_depth_image)
            
        image_filename = base_filename + '_image.npy.gz'
        fid = gzip.GzipFile(image_filename, 'w')
        np.save(fid, self.image, allow_pickle=False, fix_imports=True)
        fid.close
        
        voi_data = self.voi.serialize()
        voi_data['origin'] = voi_data['origin'].tolist()
        voi_data['axes'] = voi_data['axes'].tolist()

        if self.transform_original_to_corrected is not None:
            transform_original_to_corrected = self.transform_original_to_corrected.tolist()
        else:
            transform_original_to_corrected = None

        if self.transform_corrected_to_original is not None:
            transform_corrected_to_original = self.transform_corrected_to_original.tolist()
        else:
            transform_corrected_to_original = None
        
        data = {'visualization_filename': visualization_filename,
                'rgb_image_filename': rgb_image_filename,
                'camera_depth_image_filename': camera_depth_image_filename,
                'image_filename': image_filename,
                'image.dtype': str(self.image.dtype),
                'image.shape': list(self.image.shape),
                'np.max(image)': max_pix, 
                'm_per_pix': self.m_per_pix,
                'm_per_height_unit': self.m_per_height_unit,
                'voi_data': voi_data,
                'image_origin': self.image_origin.tolist(),
                'transform_original_to_corrected': transform_original_to_corrected, 
                'transform_corrected_to_original': transform_corrected_to_original
        }

        with open(base_filename + '.yaml', 'w') as fid:
            yaml.dump(data, fid)

        print('Finished saving.')


    @classmethod
    def load_serialization( self, base_filename ):
        print('MaxHeightImage: Loading serialization data from base_filename =', base_filename)
        with open(base_filename + '.yaml', 'r') as fid:
            data = yaml.load(fid, Loader=yaml.FullLoader)
        
        image_filename = data['image_filename']
        fid = gzip.GzipFile(image_filename, 'r')
        image = np.load(fid)
        fid.close()

        print('MaxHeightImage: Finished loading serialization data.')

        rgb_image = None
        rgb_image_filename = data.get('rgb_image_filename')
        if rgb_image_filename is not None:
            rgb_image = cv2.imread(rgb_image_filename)
            print('MaxHeightImage: Loading RGB image.')

        camera_depth_image = None
        camera_depth_image_filename = data.get('camera_depth_image_filename')
        if camera_depth_image_filename is not None:
            camera_depth_image = cv2.imread(camera_depth_image_filename)[:,:,0]
            print('MaxHeightImage: Loading camera depth image.')

        return data, image, rgb_image, camera_depth_image


    @classmethod
    def from_file( self, base_filename ):
        data, image, rgb_image, camera_depth_image = MaxHeightImage.load_serialization(base_filename)
        
        m_per_pix = data['m_per_pix']
        m_per_height_unit = data['m_per_height_unit']
        image_origin = np.array(data['image_origin'])

        voi = VolumeOfInterest.from_serialization(data['voi_data'])

        if camera_depth_image is not None:
            use_camera_depth_image = True
        else:
            use_camera_depth_image = False
        max_height_image = MaxHeightImage(voi, m_per_pix, image.dtype, m_per_height_unit, use_camera_depth_image=use_camera_depth_image, image=image, rgb_image=rgb_image, camera_depth_image=camera_depth_image)

        transform_original_to_corrected = data.get('transform_original_to_corrected', None)
        transform_corrected_to_original = data.get('transform_corrected_to_original', None)

        if transform_original_to_corrected is not None:
            transform_original_to_corrected = np.array(transform_original_to_corrected)

        if transform_corrected_to_original is not None:
            transform_corrected_to_original = np.array(transform_corrected_to_original)
        
        max_height_image.transform_original_to_corrected = transform_original_to_corrected
        max_height_image.transform_corrected_to_original = transform_corrected_to_original

        return max_height_image


    def to_points(self, colormap=None):
        
        h, w = self.image.shape
        max_num_points = w * h
        points = np.zeros((max_num_points,),
                          dtype=[
                              ('x', np.float32),
                              ('y', np.float32),
                              ('z', np.float32)])

        points_in_image_to_voi = np.identity(4)
        points_in_image_to_voi[:3, 3] = self.image_origin
        points_in_image_to_frame_id_mat = np.matmul(self.voi.points_in_voi_to_frame_id_mat, points_in_image_to_voi)
        
        if self.transform_corrected_to_original is not None: 
            points_in_image_to_frame_id_mat = np.matmul(points_in_image_to_frame_id_mat, self.transform_corrected_to_original)
        
        num_points = nh.numba_max_height_image_to_points(points_in_image_to_frame_id_mat, self.image, points, self.m_per_pix, self.m_per_height_unit)

        points = points[:num_points]

        return points


    def from_points(self, points_to_voi_mat, points):
        
        points_to_image_mat = points_to_voi_mat
        points_to_image_mat[:3,3] = points_to_image_mat[:3,3] - self.image_origin

        if self.transform_original_to_corrected is not None: 
            points_to_image_mat = np.matmul(self.transform_original_to_corrected, points_to_image_mat)
        
        if self.camera_depth_image is None: 
            nh.numba_max_height_image(points_to_image_mat, points, self.image, self.m_per_pix, self.m_per_height_unit, self.voi.x_in_m, self.voi.y_in_m, self.voi.z_in_m, verbose=False)
        else:
            print('Camera depth image with from_points command is not currently supported.')
            assert(False)

        
    def from_rgb_points(self, points_to_voi_mat, rgb_points):
        
        points_to_image_mat = points_to_voi_mat
        points_to_image_mat[:3,3] = points_to_image_mat[:3,3] - self.image_origin
        
        if self.transform_original_to_corrected is not None: 
            points_to_image_mat = np.matmul(self.transform_original_to_corrected, points_to_image_mat)

        s0, s1 = self.image.shape
        if ((self.rgb_image is None) or
            (self.rgb_image.shape != (s0, s1, 3)) or
            (self.rgb_image.dtype != np.uint8)):
            s = self.image.shape
            self.rgb_image = np.zeros(s[:2] + (3,), np.uint8)

        if self.camera_depth_image is None: 
            nh.numba_max_height_and_rgb_images(points_to_image_mat, rgb_points, self.image, self.rgb_image, self.m_per_pix, self.m_per_height_unit, self.voi.x_in_m, self.voi.y_in_m, self.voi.z_in_m, verbose=False)
        else:
            nh.numba_max_height_and_rgb_and_camera_depth_images(points_to_image_mat, rgb_points, self.image, self.rgb_image, self.camera_depth_image, self.m_per_pix, self.m_per_height_unit, self.voi.x_in_m, self.voi.y_in_m, self.voi.z_in_m, verbose=False)


