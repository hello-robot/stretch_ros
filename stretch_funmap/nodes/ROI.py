#! /usr/bin/env python

import hello_helpers.hello_misc as hm

import rospy
import logging
import numpy as np
import tf
import tf2_ros
import time
import cv2
import cv_bridge
from math import pi, sqrt, atan2
import pyrealsense2 as rs
import hello_helpers.fit_plane as fp
import stretch_funmap.ros_max_height_image as rm
import stretch_funmap.max_height_image as mh
import stretch_funmap.segment_max_height_image as sm
import stretch_funmap.manipulation_planning as mp
from stretch_funmap.manipulation_planning import plan_surface_coverage
import ros_numpy as rn

from stretch_web_interface.srv import save_pose, recreate_pose
from std_srvs.srv import Trigger

from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from sensor_msgs.msg import Image, CameraInfo, JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class clean_ROI(hm.HelloNode):
    def __init__(self):

        hm.HelloNode.__init__(self)

        self.tool_width_m = 0.08 #0.06
        self.tool_length_m = 0.08 #0.06
        self.step_size_m = 0.04 #0.06 #0.1 #0.02
        self.min_extension_m = 0.01
        self.max_extension_m = 0.5

        self.base_step = .05

    def get_camera_intrinsics(self, camera_info):
        self.intr = rs.intrinsics()
        self.intr.width = camera_info.width
        self.intr.height = camera_info.height
        self.intr.ppx = camera_info.K[2]
        self.intr.ppy = camera_info.K[5]
        self.intr.fx = camera_info.K[0]
        self.intr.fy = camera_info.K[4]
        self.intr.model = rs.distortion.none 
        self.intr.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

    def get_depth_image(self, img):
        bridge = cv_bridge.CvBridge()
        depth_img = bridge.imgmsg_to_cv2(img)
        #depth_img = cv2.rotate(depth_img, cv2.ROTATE_90_CLOCKWISE)
        self.depth_img = np.array(depth_img, dtype=np.float32)

    def get_color_image(self, img):
        cv2.namedWindow("ROI", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("ROI", self.get_region)
        
        bridge = cv_bridge.CvBridge()
        self.color_img = bridge.imgmsg_to_cv2(img)
        #self.color_img = cv2.rotate(self.color_img, cv2.ROTATE_90_CLOCKWISE)
        self.color_img = np.asanyarray(cv2.cvtColor(self.color_img, cv2.COLOR_BGR2RGB))
        if len(self.clicked_pts) > 0:
            self.color_img = cv2.polylines(self.color_img, [np.array(self.clicked_pts, np.int32)], True, (0,255,0), 4)        
        if len(self.clicked_pts) > 2:
            self.crop_image()
        
        cv2.imshow("ROI", self.color_img)
        cv2.waitKey(1)

    def get_region(self, action, x, y, flags, param):
        
        if action == cv2.EVENT_LBUTTONDOWN:
            self.clicked_pts.append([x,y])
        
        if action == cv2.EVENT_RBUTTONDOWN:
            #self.clicked_pts.pop()
            self.get_surface(self.mask, self.depth_img, self.color_img, self.cropped_img, self.intr)

        #rospy.loginfo(self.clicked_pts)

    def crop_image(self):

        self.mask = np.zeros(self.color_img.shape[:2], np.uint8)
        self.mask = cv2.fillPoly(self.mask, [np.array(self.clicked_pts, np.int32)], (255,255,255))

        self.cropped_img = np.zeros(self.color_img.shape)
        self.cropped_img = cv2.bitwise_and(self.color_img, self.color_img, mask=self.mask)
        self.cropped_img = cv2.cvtColor(self.cropped_img, cv2.COLOR_RGB2GRAY)
        self.cropped_img = np.asanyarray(self.cropped_img)

        # cv2.imshow("Crop", self.cropped_img)
        # cv2.waitKey(1)

    def joint_states_callback(self, joint_state):
        '''
        Callback for the /stretch/joint_states topic to store the current joint states for use within the class
        '''
        self.joint_state = joint_state

    def send_twist(self, rz, dx):
        '''
        Calculates and publishes a twist message on the stretch/cmd_vel topic from arguments specifying a rotation about z or a change in x 
        position. Given a fixed angular and linear speed, the length of time to publish the cmd_vel is calculated as the desired change in 
        position divided by the fixed speed.
        '''
        angular_vel = .4 #rad/sec
        linear_vel = .05 #m/sec

        twist = Twist()

        if dx != 0:
            twist.linear.x = linear_vel
            if dx < 0:
                twist.linear.x *= -1
            t = abs(dx) / linear_vel
        else:
            twist.linear.x = 0

        if rz != 0:
            twist.angular.z = angular_vel
            if rz < 0:
                twist.angular.z *= -1
            t = abs(rz) / angular_vel
        else:
            twist.angular.z = 0
        
        start_time = rospy.Time.now()

        rospy.loginfo(twist)

        while rospy.Time.now() < start_time + rospy.Duration.from_sec(t):
            self.cmd_pub.publish(twist)
            time.sleep(.25)

    def get_surface(self, surface_mask, depth_img, color_img, cropped_img, intr):

        h_image = depth_img.copy()
        depth_region = []

        print("SCANNING SURFACE")

        for r in range(h_image.shape[0]):
            for c in range(h_image.shape[1]):
                if cropped_img[r][c] != 0.00:
                    depth = depth_img[r][c]
                    coord = rs.rs2_deproject_pixel_to_point(intr, [c,r], depth)
                    
                    point_in_camera = TransformStamped()
                    point_in_camera.header.stamp = rospy.Time.now()
                    point_in_camera.header.frame_id = "camera_depth_optical_frame"
                    point_in_camera.child_frame_id = "pt"
                    point_in_camera.transform.translation.x = coord[0]/1000.0 #coord[2]/1000.0
                    point_in_camera.transform.translation.y = coord[1]/1000.0 #-1*coord[0]/1000.0
                    point_in_camera.transform.translation.z = coord[2]/1000.0 #-1*coord[1]/1000.0
                    point_in_camera.transform.rotation.w = 1.0

                    self.static_broadcaster.sendTransform(point_in_camera)

                    self.tf_listener.waitForTransform("base_link", "pt", rospy.Time(0), rospy.Duration(5.0))
                    trans, rot = self.tf_listener.lookupTransform("base_link", "pt", rospy.Time(0))

                    h_image[r][c] = trans[2]
                    depth_region.append(trans[2])
                else:
                    h_image[r][c] = 0.0

        print("AVG. HEIGHT: ", sum(depth_region)/len(depth_region))
        colored_depth = cv2.applyColorMap((h_image*1000.0).astype(np.uint8), cv2.COLORMAP_JET)

        cv2.imshow("Depth", colored_depth)
        cv2.waitKey(1)

        self.surface_height_m = np.median(depth_region)
        print("SURFACE HEIGHT: ", self.surface_height_m)

        # Identify Obstacles
        min_obstacle_height_m = self.surface_height_m + 0.010
        obstacle_selector = h_image > min_obstacle_height_m
        obstacle_mask = np.uint8(obstacle_selector*255)

        # Open Obstacles, Erode Surface
        kernel_radius_pix = 20 #int(round(max(tool_width_pix, tool_length_pix)/2.0))
        kernel_width_pix = 1 + (2 * kernel_radius_pix)
        iterations = 1
        kernel = np.zeros((kernel_width_pix, kernel_width_pix), np.uint8)
        cv2.circle(kernel, (kernel_radius_pix, kernel_radius_pix), kernel_radius_pix, 255, -1)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        obstacle_mask = cv2.dilate(obstacle_mask, kernel, iterations=iterations)
        #surface_mask = cv2.erode(surface_mask, kernel, iterations=iterations)
        obstacle_mask = cv2.bitwise_not(obstacle_mask)



        self.masked_img = cv2.bitwise_and(cropped_img, cropped_img, mask=obstacle_mask)
        cv2.imshow("Final Masked", self.masked_img)
        cv2.waitKey(1)

        wrist_ext_steps = self.plan_path()
        self.clean_surface(wrist_ext_steps)

    def plan_path(self):

        self.path_img = self.masked_img.copy()

        corners = cv2.goodFeaturesToTrack(self.masked_img, 8, 0.01, 10)
        print(corners)
        corner_list = []
        for corner in corners:
            corner_list.append([corner[0][0], corner[0][1]])
        print(corner_list)
        corner_list = sorted(corner_list, key=lambda k:[k[1], k[0]])
        print(corner_list)
        corner_list = [corner_list[0], corner_list[1], corner_list[-2], corner_list[-1]]
        print(corner_list)

        if corner_list[0][0] < corner_list[1][0]:
            px_corner1 = tuple(corner_list[0])
            px_corner2 = tuple(corner_list[1])
        else:
            px_corner2 = tuple(corner_list[0])
            px_corner1 = tuple(corner_list[1])
        if corner_list[2][0] < corner_list[3][0]:
            px_corner3 = tuple(corner_list[2])
            px_corner4 = tuple(corner_list[3]) 
        else:
            px_corner4 = tuple(corner_list[2])
            px_corner3 = tuple(corner_list[3]) 

        print(px_corner1, px_corner2, px_corner3, px_corner4) 

        self.path_img = cv2.circle(self.path_img, px_corner1, 3, 100, -1)
        depth = self.depth_img[int(px_corner1[1])][int(px_corner1[0])]
        corner1 = rs.rs2_deproject_pixel_to_point(self.intr, px_corner1, depth)

        self.path_img = cv2.circle(self.path_img, px_corner2, 3, 150, -1)
        depth = self.depth_img[int(px_corner2[1])][int(px_corner2[0])]
        corner2 = rs.rs2_deproject_pixel_to_point(self.intr, px_corner2, depth)

        self.path_img = cv2.circle(self.path_img, px_corner3, 3, 200, -1)
        depth = self.depth_img[int(px_corner3[1])][int(px_corner3[0])]
        corner3 = rs.rs2_deproject_pixel_to_point(self.intr, px_corner3, depth)
        
        self.path_img = cv2.circle(self.path_img, px_corner4, 3, 250, -1)
        depth = self.depth_img[int(px_corner4[1])][int(px_corner4[0])]
        corner4 = rs.rs2_deproject_pixel_to_point(self.intr, px_corner4, depth)

        self.first_pt = "corner1"

        self.corners_in_base = []
        corner_tf_list = []
        string = ["corner1", "corner2", "corner3", "corner4"]
        i = 0
        for corner in [corner1, corner2, corner3, corner4]:
            
            corner_tf = TransformStamped()
            corner_tf.header.stamp = rospy.Time.now()
            corner_tf.header.frame_id = "camera_depth_optical_frame"
            corner_tf.child_frame_id = string[i]
            corner_tf.transform.translation.x = corner[0]/1000.0
            corner_tf.transform.translation.y = corner[1]/1000.0
            corner_tf.transform.translation.z = corner[2]/1000.0
            corner_tf.transform.rotation.w = 1.0

            corner_tf_list.append(corner_tf)

            i += 1

        self.static_broadcaster.sendTransform(corner_tf_list)

        i = 0
        for corner in [corner1, corner2, corner3, corner4]:
            
            self.tf_listener.waitForTransform("base_link", string[i], rospy.Time(0), rospy.Duration(5.0))
            trans, rot = self.tf_listener.lookupTransform("base_link", string[i], rospy.Time(0))           
            self.corners_in_base.append([trans[0],trans[1]])

            i += 1

        x_axis = [1,0]
        cleaning_axis1 = [self.corners_in_base[2][0] - self.corners_in_base[0][0],self.corners_in_base[2][1] - self.corners_in_base[0][1]]
        cleaning_axis2 = [self.corners_in_base[3][0] - self.corners_in_base[1][0],self.corners_in_base[3][1] - self.corners_in_base[1][1]]
 
        angle1 = self.angle(cleaning_axis1, x_axis)
        angle2 = self.angle(cleaning_axis2, x_axis)

        print(angle1, angle2)
        rot = min(angle1, angle2) #(angle1 + angle2) / 2.0
        print('********* rotate parallel to surface = {0} degrees **************'.format(np.rad2deg(rot)))
        self.send_twist(rot, 0)

        if abs(rot) < 80:
            x_distance_m = max(self.corners_in_base[2][0] - self.corners_in_base[0][0], self.corners_in_base[3][0] - self.corners_in_base[1][0])
            x_distance_px = max(px_corner3[1] - px_corner1[1], px_corner4[1] - px_corner2[1])
            print("x_distance_m: ", x_distance_m)
            print("x_distance_px: ", x_distance_px)
        else:
            x_distance_m = max(abs(corners[1][0] - corners[0][0]), abs(corners[4][0] - corners[3][0]))
            x_distance_px = max(px_corner3[1] - px_corner1[1], px_corner4[1] - px_corner2[1])
            print("x_distance_m: ", x_distance_m)
            print("x_distance_px: ", x_distance_px)            

        self.num_steps = int(x_distance_m / self.base_step)
        step_px = int(x_distance_px / self.num_steps)

        wrist_ext_steps = []
        r = int(px_corner1[1])

        forward = False
        for i in range(self.num_steps):

            if len(np.where(self.masked_img[r] > 0)[0]) < 1:
                wrist_ext_steps.append(0.0) 
                r = int(r + step_px)
                forward = not forward
                continue

            if forward:
                c1 = min(np.where(self.masked_img[r] > 0)[0])
                c2 = max(np.where(self.masked_img[r] > 0)[0])
                for c in range(c1, c2):
                    if self.masked_img[r][c] == 0:
                        c2 = c
                        break
            else:
                c1 = max(np.where(self.masked_img[r] > 0)[0])
                c2 = min(np.where(self.masked_img[r] > 0)[0])
                for c in range(c1, c2, -1):
                    if self.masked_img[r][c] == 0:
                        c1 = c
                        break

            depth1 = self.depth_img[r][c1]
            depth2 = self.depth_img[r][c2]
            pt1 = rs.rs2_deproject_pixel_to_point(self.intr, [c1,r], depth)
            pt2 = rs.rs2_deproject_pixel_to_point(self.intr, [c2,r], depth)

            t1 = TransformStamped()
            t1.header.stamp = rospy.Time.now()
            t1.header.frame_id = "camera_depth_optical_frame"
            t1.child_frame_id = "pt1"
            t1.transform.translation.x = pt1[0]/1000.0
            t1.transform.translation.y = pt1[1]/1000.0
            t1.transform.translation.z = pt1[2]/1000.0
            t1.transform.rotation.w = 1.0

            t2 = TransformStamped()
            t2.header.stamp = rospy.Time.now()
            t2.header.frame_id = "camera_depth_optical_frame"
            t2.child_frame_id = "pt2"
            t2.transform.translation.x = pt2[0]/1000.0
            t2.transform.translation.y = pt2[1]/1000.0
            t2.transform.translation.z = pt2[2]/1000.0
            t2.transform.rotation.w = 1.0

            self.static_broadcaster.sendTransform([t1, t2])

            self.tf_listener.waitForTransform("base_link", "pt1", rospy.Time(0), rospy.Duration(5.0))
            trans1, rot1 = self.tf_listener.lookupTransform("base_link", "pt1", rospy.Time(0))

            self.tf_listener.waitForTransform("base_link", "pt2", rospy.Time(0), rospy.Duration(5.0))
            trans2, rot2 = self.tf_listener.lookupTransform("base_link", "pt2", rospy.Time(0))

            print(r, c1)
            print(trans1)
            print(r, c2)
            print(trans2)
            ext = abs(trans2[1]) - abs(trans1[1])

            wrist_ext_steps.append(ext) 

            self.path_img = cv2.circle(self.path_img, (c1, r), 1, 255, -1)
            self.path_img = cv2.circle(self.path_img, (c2, r), 1, 255, -1)

            r = int(r + step_px)
            forward = not forward

        print("********** PLAN ***************")
        print(wrist_ext_steps)

        cv2.imshow("Path", self.path_img)
        cv2.waitKey(1)

        return wrist_ext_steps        

    def angle(self, v1, v2):
        norm1 = v1 / np.linalg.norm(v1)
        norm2 = v2 / np.linalg.norm(v2)

        dot = norm1[0] * norm2[0] + norm1[1] * norm2[1]

        return np.arccos(dot)

    def clean_surface(self, wrist_ext_steps):
        
        ## Initial Pose ##

        print('********* lift_to_surface_m = {0} **************'.format(self.surface_height_m))
        pose = {'joint_lift': self.surface_height_m + .15}
        self.move_to_pose(pose)

        self.tf_listener.waitForTransform("link_wrist_yaw","corner1", rospy.Time(0), rospy.Duration(5.0))
        trans, rot_quat = self.tf_listener.lookupTransform("link_wrist_yaw","corner1", rospy.Time(0))
        print('********* translate to first point = {0} m**************'.format(trans[0]))
        self.send_twist(0,-1*(trans[0] + .05))

        pose = {'joint_wrist_yaw': 0}
        self.move_to_pose(pose)

        ## Start ##
        
        print('********* extend_to_surface_m = {0} **************'.format(trans[1]))
        new_value = -1*trans[1] - self.tool_length_m

        if new_value > self.max_extension_m:
            new_value = self.max_extension_m

        if new_value < self.min_extension_m:
            new_value = self.min_extension_m
        pose = {'wrist_extension': new_value}
        self.move_to_pose(pose)

        print('********* lower_to_surface_m = {0} **************'.format(self.surface_height_m))
        pose = {'joint_lift': self.surface_height_m + .02}
        self.move_to_pose(pose)

        joint_index = self.joint_state.name.index('wrist_extension')

        steps = 0
        for delta in wrist_ext_steps:
            joint_value = self.joint_state.position[joint_index]
            new_value = joint_value + delta

            if new_value > self.max_extension_m:
                new_value = self.max_extension_m

            if new_value < self.min_extension_m:
                new_value = self.min_extension_m

            pose = {'wrist_extension': new_value}
            self.move_to_pose(pose)

            if steps < (len(wrist_ext_steps) - 1):
                self.send_twist(0,self.base_step)

            steps += 1
            self.rate.sleep() 

    def main(self):

        hm.HelloNode.main(self, 'ROI', 'ROI', wait_for_first_pointcloud=False)

        self.rate = rospy.Rate(rospy.get_param('~rate', 15.0))
        
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.get_depth_image)
        rospy.Subscriber("/camera/color/image_raw", Image, self.get_color_image)
        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.get_camera_intrinsics)

        self.bridge = cv_bridge.CvBridge()        
        self.clicked_pts = []

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.tf2_buffer = tf2_ros.Buffer(rospy.Duration(2.0))

        self.cmd_pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_states_callback)
        
        # self.trigger_clean_surface_service = rospy.Service('/clean_surface/trigger_clean_surface',
        #                                                    Trigger,
        #                                                    self.trigger_clean_surface_callback)

        rospy.wait_for_service('/funmap/trigger_reach_until_contact')
        rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_reach_until_contact.')
        self.trigger_reach_until_contact_service = rospy.ServiceProxy('/funmap/trigger_reach_until_contact', Trigger)

        rospy.wait_for_service('/funmap/trigger_lower_until_contact')
        rospy.loginfo('Node ' + self.node_name + ' connected to /funmap/trigger_lower_until_contact.')
        self.trigger_lower_until_contact_service = rospy.ServiceProxy('/funmap/trigger_lower_until_contact', Trigger)

        while not rospy.is_shutdown():
            self.rate.sleep()        
    

if __name__ == '__main__':
    
    node = clean_ROI()
    node.main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        cv2.destroyAllWindows()
        print('interrupt received, so shutting down')
