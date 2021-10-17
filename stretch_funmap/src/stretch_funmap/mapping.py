#!/usr/bin/env python3

import numpy as np
import ros_numpy as rn
import stretch_funmap.ros_max_height_image as rm
from actionlib_msgs.msg import GoalStatus
import rospy
import hello_helpers.hello_misc as hm
import stretch_funmap.ros_max_height_image as rm
import ros_numpy
import yaml
import stretch_funmap.navigation_planning as na
import time
import cv2
import copy
import scipy.ndimage as nd
import stretch_funmap.merge_maps as mm
import tf_conversions
import stretch_funmap.segment_max_height_image as sm


def stow_and_lower_arm(node):
    pose = {'joint_gripper_finger_left': -0.15}
    node.move_to_pose(pose)
    pose = {'wrist_extension': 0.01}
    node.move_to_pose(pose)

    # gripper backwards stow
    pose = {'joint_wrist_yaw': 3.3}
    
    # gripper forward stow needs a better forward range of motion to work well
    node.move_to_pose(pose)
    
    # avoid blocking the laser range finder with the gripper
    pose = {'joint_lift': 0.22}
    node.move_to_pose(pose)
    return 'lowered'


def draw_robot_pose(robot_xya_pix, image, m_per_pix, color=(0, 0, 255)):
    radius = 10
    x = int(round(robot_xya_pix[0]))
    y = int(round(robot_xya_pix[1]))
    a = robot_xya_pix[2]    
    print('robot (x, y, a_deg) = ({0}, {1}, {2})'.format(x,y, 180.0*(a/np.pi)))
    color_image_input = (len(image.shape) == 3)
    if color_image_input: 
        cv2.circle(image, (x,y), radius, color, 1)
    else:
        if len(color) == 1: 
            cv2.circle(image, (x,y), radius, color, 1)
        else:
            cv2.circle(image, (x,y), radius, 255, 1)
    direction_length_m = 0.25
    direction_length_pix = direction_length_m / m_per_pix
    x2 = int(round(x + (direction_length_pix * np.cos(a))))
    y2 = int(round(y + (direction_length_pix * -np.sin(a))))
    if color_image_input: 
        cv2.line(image, (x, y), (x2, y2), color, 2)
    else:
        if len(color) == 1: 
            cv2.line(image, (x, y), (x2, y2), color, 2)
        else: 
            cv2.line(image, (x, y), (x2, y2), 255, 2)
            
def display_head_scan(title, head_scan, scale_divisor=None, robot_xya_pix_list=None):
    image = head_scan.max_height_im.image
    h, w = image.shape
    color_im = np.zeros((h, w, 3), np.uint8)
    color_im[:,:,0] = image
    color_im[:,:,1] = image
    color_im[:,:,2] = image
    draw_robot_pose([head_scan.robot_xy_pix[0],
                     head_scan.robot_xy_pix[1],
                     head_scan.robot_ang_rad],
                    color_im,
                    head_scan.max_height_im.m_per_pix)
    if robot_xya_pix_list is not None:
        for xya in robot_xya_pix_list: 
            draw_robot_pose(xya, 
                            color_im,
                            head_scan.max_height_im.m_per_pix,
                            color=(0, 255, 0))
    if scale_divisor is None:
        cv2.imshow(title, color_im)
    else:
        # scale the map so that it can be viewed on a small monitor
        nh = h//scale_divisor
        nw = w//scale_divisor
        color_im = cv2.resize(color_im, (nw, nh))
        cv2.imshow(title, color_im)


def localize_with_reduced_images(head_scan, merged_map, global_localization=True, divisor=6, small_search=False):
    # The robot has not been localized with respect to the
    # current map. This attempts to localize the robot on
    # the map by reducing the sizes of the scan and the
    # map in order to more efficiently search for a match
    # globally. 

    # Currently, because this code is under development,
    # it only localizes and does not merge the new scan
    # into the current map.

    if global_localization: 
        full_localization = True
        grid_search = True
        recenter = True
    else:
        full_localization = False
        grid_search = False
        recenter = False
            
    hs_0 = copy.deepcopy(merged_map)
    hs_1 = copy.deepcopy(head_scan)

    mhi_0 = hs_0.max_height_im
    mhi_1 = hs_1.max_height_im

    h, w = mhi_0.image.shape
    
    nh = h//divisor
    nw = w//divisor

    mhi_0.image = cv2.resize(mhi_0.image, (nw, nh))
    mhi_0.camera_depth_image = cv2.resize(mhi_0.camera_depth_image, (nw, nh))
    mhi_0.rgb_image = None
    hs_0.robot_xy_pix = hs_0.robot_xy_pix/divisor
    scale_mat = np.identity(4)
    scale_mat[0,0] = divisor
    scale_mat[1,1] = divisor
    hs_0.image_to_map_mat = np.matmul(hs_0.image_to_map_mat, scale_mat)

    mhi_1.image = cv2.resize(mhi_1.image, (nw, nh))
    mhi_1.camera_depth_image = cv2.resize(mhi_1.camera_depth_image, (nw, nh))
    mhi_1.rgb_image = None
    hs_1.robot_xy_pix = hs_1.robot_xy_pix/divisor
    hs_1.image_to_map_mat = np.matmul(hs_1.image_to_map_mat, scale_mat)

    start_time = time.time()
    h, w = mhi_1.image.shape
    min_dim = min(w,h)

    if recenter:
        crop = False
        if not crop:
            mask = mhi_1.image > 0
            center_y, center_x = nd.measurements.center_of_mass(mask)
            h,w = mhi_1.image.shape
            translation_matrix = np.identity(3)
            delta_xy = np.array([(w/2)-center_x, (h/2)-center_y])
            translation_matrix[:2, 2] = delta_xy
            translation_matrix = translation_matrix[:2, :]

            mhi_1.image = cv2.warpAffine(mhi_1.image, translation_matrix, mhi_1.image.shape, flags=cv2.INTER_NEAREST)
            mhi_1.camera_depth_image = cv2.warpAffine(mhi_1.camera_depth_image, translation_matrix,
                                                      mhi_1.camera_depth_image.shape, flags=cv2.INTER_NEAREST)

            hs_1.robot_xy_pix = hs_1.robot_xy_pix + delta_xy
            trans_mat = np.identity(4)
            trans_mat[:2, 3] = -delta_xy
            hs_1.image_to_map_mat = np.matmul(hs_1.image_to_map_mat, trans_mat)
        else: 
            m_per_pix = mhi_1.m_per_pix * divisor
            mask = mhi_1.image > 0
            center_y, center_x = hs_1.robot_xy_pix
            h,w = mhi_1.image.shape
            translation_matrix = np.identity(3)
            delta_xy = np.array([(w/2)-center_x, (h/2)-center_y])
            translation_matrix[:2, 2] = delta_xy
            translation_matrix = translation_matrix[:2, :]

            mhi_1.image = cv2.warpAffine(mhi_1.image, translation_matrix, mhi_1.image.shape, flags=cv2.INTER_NEAREST)
            mhi_1.camera_depth_image = cv2.warpAffine(mhi_1.camera_depth_image, translation_matrix,
                                                      mhi_1.camera_depth_image.shape, flags=cv2.INTER_NEAREST)

            keep_view_distance_m = 4.0
            subwindow_size = 2 * (keep_view_distance_m/m_per_pix)
            x0 = int(round((w - subwindow_size) / 2))
            x1 = int(round(x0 + subwindow_size))
            y0 = int(round(((h - subwindow_size) / 2)))
            y1 = int(round(y0 + subwindow_size))
            if (x0 > 0) and (y0 > 0): 
                mhi_1.image = mhi_1.image[y0:y1, x0:x1].copy()
                mhi_1.camera_depth_image = mhi_1.camera_depth_image[y0:y1, x0:x1].copy()
                delta_xy = delta_xy - np.array([x0, y0])
                
            hs_1.robot_xy_pix = hs_1.robot_xy_pix + delta_xy
            trans_mat = np.identity(4)
            trans_mat[:2, 3] = -delta_xy
            hs_1.image_to_map_mat = np.matmul(hs_1.image_to_map_mat, trans_mat)

    if global_localization:
        # initialize without doing anything (no transform performed)
        init_target = [w/2, h/2, 0.0]
    else:
        init_target = None

    affine_matrix, original_robot_map_pose, corrected_robot_map_pose = mm.estimate_scan_1_to_scan_2_transform(hs_1, hs_0,
                                                                                                              display_on=False,
                                                                                                              show_unaligned=False,
                                                                                                              full_localization=full_localization,
                                                                                                              init_target=init_target,
                                                                                                              grid_search=grid_search,
                                                                                                              small_search=small_search)

    scan_x, scan_y = hs_1.robot_xy_pix
    scan_a = hs_1.robot_ang_rad
    map_im_x, map_im_y, map_im_a = mm.transform_xya_to_xya_2d(affine_matrix,
                                                              scan_x, scan_y, scan_a)
    # Due to matching being performed with scaled images
    map_im_x = map_im_x * divisor
    map_im_y = map_im_y * divisor
    # Due to image y-axis sign flip
    map_im_a = -map_im_a

    im_to_map_mat = merged_map.image_to_map_mat
    map_x, map_y, map_ang_rad = mm.transform_xya_to_xya_3d(im_to_map_mat,
                                                           map_im_x, map_im_y, map_im_a)
    map_xy_1 = np.array([map_x, map_y])
    map_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, map_ang_rad)
    print('map_xy_1 =', map_xy_1)
    print('map_ang_rad =', map_ang_rad)

    x, y, a = mm.transform_xya_to_xya_3d(im_to_map_mat,
                                         head_scan.robot_xy_pix[0],
                                         head_scan.robot_xy_pix[1],
                                         head_scan.robot_ang_rad)

    original_robot_map_frame_pose = [x, y, a]
    corrected_robot_map_frame_pose = [map_x, map_y, map_ang_rad]

    original_robot_map_image_pose = [head_scan.robot_xy_pix[0],
                                     head_scan.robot_xy_pix[1],
                                     head_scan.robot_ang_rad]
    corrected_robot_map_image_pose = [map_im_x, map_im_y, map_im_a]

    end_time = time.time()
    total_time = end_time - start_time
    print('Total time to match to the loaded map =', total_time)

    scaled_merged_map = hs_0
    scaled_scan = hs_1 
    
    return original_robot_map_frame_pose, corrected_robot_map_frame_pose, original_robot_map_image_pose, corrected_robot_map_image_pose, scaled_scan, scaled_merged_map


        
class HeadScan:
    def __init__(self, max_height_im=None, voi_side_m=8.0, voi_origin_m=None):
        if max_height_im is not None:
            self.max_height_im = max_height_im
        else:
            # How to best set this volume of interest (VOI) merits further
            # consideration. Note that representing a smaller range of heights
            # results in higher height resolution when using np.uint8
            # pixels. For this VOI, 0.0 should be the nominal ground height
            # achieved via calibration.

            # Set to approximately the height of the D435i. This should result
            # in the volume of interest (VOI) containing the highest
            # manipulable surfaces. Also, when the top of the viewing frustum
            # is parallel to the ground it will be at or close to the top of
            # the volume of interest.
            
            robot_head_above_ground = 1.13
            
            # How far below the expected floor height the volume of interest
            # should extend is less clear. Sunken living rooms and staircases
            # can go well below the floor and a standard stair step should be
            # less than 20cm tall (0.2 m below the floor). However, the robot
            # should not go into these areas. For now, the volume of interest
            # (VOI) will contain points that the robot can potentially reach
            # its arm over or drive over (traverse). This implies that
            # unobserved points on the floor should be treated with great
            # caution, since they might be points significantly below the
            # floor that should not be traversed. For now, the robot will not
            # represent ramps that it might safely descend. It should be able
            # to represent floor points that look slightly lower due to noise
            # that can vary with floor type and small calibration errors. It
            # should be able to represent small traversable depressions in the
            # floor. However, there is a risk that points that are too low
            # will be classified as traversable floor. This risk is mitigated
            # by separate point cloud based obstacle detection while moving
            # and cliff sensors.
            
            lowest_distance_below_ground = 0.05 #5cm
            
            total_height = robot_head_above_ground + lowest_distance_below_ground
            # 8m x 8m region 
            voi_side_m = voi_side_m
            voi_axes = np.identity(3)
            if voi_origin_m is None: 
                voi_origin = np.array([-voi_side_m/2.0, -voi_side_m/2.0, -lowest_distance_below_ground])
            voi = rm.ROSVolumeOfInterest('map', voi_origin, voi_axes, voi_side_m, voi_side_m, total_height)

            m_per_pix = 0.006
            pixel_dtype = np.uint8
            
            self.max_height_im = rm.ROSMaxHeightImage(voi, m_per_pix, pixel_dtype, use_camera_depth_image=True)
            self.max_height_im.create_blank_rgb_image()
            
        self.max_height_im.print_info()

        
    def make_robot_footprint_unobserved(self):
        # replace robot points with unobserved points
        self.max_height_im.make_robot_footprint_unobserved(self.robot_xy_pix[0], self.robot_xy_pix[1], self.robot_ang_rad)

    def make_robot_mast_blind_spot_unobserved(self):
        # replace robot points with unobserved points
        self.max_height_im.make_robot_mast_blind_spot_unobserved(self.robot_xy_pix[0], self.robot_xy_pix[1], self.robot_ang_rad)
        
    def capture_point_clouds(self, node, pose, capture_params):
        head_settle_time = capture_params['head_settle_time']
        num_point_clouds_per_pan_ang = capture_params['num_point_clouds_per_pan_ang']
        time_between_point_clouds = capture_params['time_between_point_clouds']
        fast_scan = capture_params.get('fast_scan', False)

        if fast_scan:
            head_settle_time = head_settle_time
            num_point_clouds_per_pan_ang = 1
            time_between_point_clouds = time_between_point_clouds
        
        node.move_to_pose(pose)
        rospy.sleep(head_settle_time)
        settle_time = rospy.Time.now()
        prev_cloud_time = None
        num_point_clouds = 0
        # Consider using time stamps to make decisions, instead of
        # hard coded sleep times, as found in the head calibration
        # data collection code. The main issue is that the robot
        # needs time to mechanically settle in addition to sensor
        # timing considerations.
        not_finished = num_point_clouds < num_point_clouds_per_pan_ang
        while not_finished:
            cloud_time = node.point_cloud.header.stamp
            cloud_frame = node.point_cloud.header.frame_id
            point_cloud = ros_numpy.numpify(node.point_cloud)
            if (cloud_time is not None) and (cloud_time != prev_cloud_time) and (cloud_time >= settle_time): 
                only_xyz = False
                if only_xyz:
                    xyz = ros_numpy.point_cloud2.get_xyz_points(point_cloud)
                    self.max_height_im.from_points_with_tf2(xyz, cloud_frame, node.tf2_buffer)
                else: 
                    rgb_points = ros_numpy.point_cloud2.split_rgb_field(point_cloud)
                    self.max_height_im.from_rgb_points_with_tf2(rgb_points, cloud_frame, node.tf2_buffer)
                num_point_clouds += 1
                prev_cloud_time = cloud_time
            not_finished = num_point_clouds < num_point_clouds_per_pan_ang
            if not_finished: 
                rospy.sleep(time_between_point_clouds)


    def execute(self, head_tilt, far_left_pan, far_right_pan, num_pan_steps, capture_params, node, look_at_self=True):
        scan_start_time = time.time()

        pose = {'joint_head_pan': far_right_pan, 'joint_head_tilt': head_tilt}
        node.move_to_pose(pose)
        
        pan_left = np.linspace(far_right_pan, far_left_pan, num_pan_steps)

        for pan_ang in pan_left:
            pose = {'joint_head_pan': pan_ang}
            self.capture_point_clouds(node, pose, capture_params)
            
        # look at the ground right around the robot to detect any
        # nearby obstacles

        if look_at_self:
            # Attempt to pick a head pose that sees around the robot,
            # but doesn't see the mast, which can introduce noise.
            head_tilt = -1.2
            head_pan = 0.1
            pose = {'joint_head_pan': head_pan, 'joint_head_tilt': head_tilt}
            self.capture_point_clouds(node, pose, capture_params)

        scan_end_time = time.time()
        scan_duration = scan_end_time - scan_start_time
        rospy.loginfo('The head scan took {0} seconds.'.format(scan_duration))
            
        #####################################
        # record robot pose information and potentially useful transformations
        self.robot_xy_pix, self.robot_ang_rad, self.timestamp = self.max_height_im.get_robot_pose_in_image(node.tf2_buffer)

        # Should only need three of these transforms, since the other
        # three should be obtainable via matrix inversion. Variation
        # in time could result in small differences due to encoder
        # noise.
        self.base_link_to_image_mat, timestamp = self.max_height_im.get_points_to_image_mat('base_link', node.tf2_buffer)
        self.base_link_to_map_mat, timestamp = hm.get_p1_to_p2_matrix('base_link', 'map', node.tf2_buffer)
        self.image_to_map_mat, timestamp = self.max_height_im.get_image_to_points_mat('map', node.tf2_buffer)
        self.image_to_base_link_mat, timestamp = self.max_height_im.get_image_to_points_mat('base_link', node.tf2_buffer)
        self.map_to_image_mat, timestamp = self.max_height_im.get_points_to_image_mat('map', node.tf2_buffer)
        self.map_to_base_mat, timestamp = hm.get_p1_to_p2_matrix('map', 'base_link', node.tf2_buffer)

        self.make_robot_mast_blind_spot_unobserved()
        self.make_robot_footprint_unobserved()
                
        
    def execute_full(self, node, fast_scan=False):
        far_right_pan = -3.6 
        far_left_pan = 1.45 
        head_tilt = -0.8 
        num_pan_steps = 7 

        if fast_scan:
            num_pan_steps = 5 

        capture_params = {
            'fast_scan': fast_scan,
            'head_settle_time': 0.5,
            'num_point_clouds_per_pan_ang': 10, # low numbers may not be effective for some surfaces and environments
            'time_between_point_clouds': 1.0/15.0 # point clouds at 15 Hz, so this should help obtain distinct clouds
        }

        self.execute(head_tilt, far_left_pan, far_right_pan, num_pan_steps, capture_params, node)


    def execute_front(self, node, fast_scan=False):
        far_right_pan = -1.2 
        far_left_pan = 1.2 
        head_tilt = -0.8
        num_pan_steps = 3
        
        capture_params = {
            'fast_scan': fast_scan,
            'head_settle_time': 0.5,
            'num_point_clouds_per_pan_ang': 10, # low numbers may not be effective for some surfaces and environments
            'time_between_point_clouds': 1.0/15.0 # point clouds at 15 Hz, so this should help obtain distinct clouds
        }

        self.execute(head_tilt, far_left_pan, far_right_pan, num_pan_steps, capture_params, node)

    
    def execute_minimal(self, node, fast_scan=False):
        far_right_pan = 0.1
        far_left_pan = 0.1
        head_tilt = -0.8
        num_pan_steps = 1

        look_at_self = True
        
        capture_params = {
            'fast_scan': fast_scan,
            'head_settle_time': 0.5,
            'num_point_clouds_per_pan_ang': 10, # low numbers may not be effective for some surfaces and environments
            'time_between_point_clouds': 1.0/15.0 # point clouds at 15 Hz, so this should help obtain distinct clouds
        }

        self.execute(head_tilt, far_left_pan, far_right_pan, num_pan_steps, capture_params, node, look_at_self)
        
        
    def save( self, base_filename, save_visualization=True ):
        print('HeadScan: Saving to base_filename =', base_filename)
        # save scan to disk
        max_height_image_base_filename = base_filename + '_mhi'
        self.max_height_im.save(max_height_image_base_filename)

        if "tolist" in dir(self.robot_ang_rad):
            robot_ang_rad = self.robot_ang_rad.tolist()
        else:
            robot_ang_rad = self.robot_ang_rad
        data = {'max_height_image_base_filename' : max_height_image_base_filename,
                'robot_xy_pix' : self.robot_xy_pix.tolist(),
                'robot_ang_rad' : robot_ang_rad,
                'timestamp' : {'secs':self.timestamp.secs, 'nsecs':self.timestamp.nsecs},
                'base_link_to_image_mat' : self.base_link_to_image_mat.tolist(), 
                'base_link_to_map_mat' : self.base_link_to_map_mat.tolist(), 
                'image_to_map_mat' : self.image_to_map_mat.tolist(), 
                'image_to_base_link_mat' : self.image_to_base_link_mat.tolist(), 
                'map_to_image_mat' : self.map_to_image_mat.tolist(), 
                'map_to_base_mat' : self.map_to_base_mat.tolist()}
        
        with open(base_filename + '.yaml', 'w') as fid:
            yaml.dump(data, fid)
        print('Finished saving.')

        
    @classmethod
    def from_file(self, base_filename):
        print('HeadScan.from_file: base_filename =', base_filename)
        with open(base_filename + '.yaml', 'r') as fid:
            data = yaml.load(fid, Loader=yaml.FullLoader)

        print('data =', data)
        max_height_image_base_filename = data['max_height_image_base_filename']
        max_height_image = rm.ROSMaxHeightImage.from_file(max_height_image_base_filename)
        head_scan = HeadScan(max_height_image)

        head_scan.robot_xy_pix = np.array(data['robot_xy_pix'])
        head_scan.robot_ang_rad = data['robot_ang_rad']
        head_scan.timestamp = rospy.Time()
        head_scan.timestamp.set(data['timestamp']['secs'], data['timestamp']['nsecs'])
        head_scan.base_link_to_image_mat = np.array(data['base_link_to_image_mat'])
        head_scan.base_link_to_map_mat = np.array(data['base_link_to_map_mat']) 
        head_scan.image_to_map_mat = np.array(data['image_to_map_mat'])
        head_scan.image_to_base_link_mat = np.array(data['image_to_base_link_mat'])
        head_scan.map_to_image_mat = np.array(data['map_to_image_mat'])
        head_scan.map_to_base_mat = np.array(data['map_to_base_mat'])

        return head_scan
