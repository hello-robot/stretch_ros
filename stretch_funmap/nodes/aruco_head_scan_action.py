#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Transform, TransformStamped
import ros_numpy
import numpy as np
import tf2_ros

import actionlib
from stretch_core.msg import ArucoHeadScanAction, ArucoHeadScanGoal, ArucoHeadScanFeedback, ArucoHeadScanResult
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
import hello_helpers.hello_misc as hm

import stretch_funmap.mapping as ma
import time

def create_map_to_odom_transform(t_mat):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'map'
    t.child_frame_id = 'odom'
    t.transform = ros_numpy.msgify(Transform, t_mat)
    return t

class ArucoHeadScan(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'aruco_head_scan', 'aruco_head_scan', wait_for_first_pointcloud=False)
        self.server = actionlib.SimpleActionServer('ArucoHeadScan', ArucoHeadScanAction, self.execute_cb, False)
        self.goal = ArucoHeadScanGoal()
        self.feedback = ArucoHeadScanFeedback()
        self.result = ArucoHeadScanResult()
        self.aruco_marker_array = rospy.Subscriber('aruco/marker_array', MarkerArray, self.aruco_callback)
        self.joint_states_sub = rospy.Subscriber('/stretch/joint_states', JointState, self.joint_state_callback)
        self.server.start()
        self.aruco_id = 1000 # Placeholder value, can never be true
        self.aruco_found = False
        self.markers = MarkerArray().markers
        self.joint_state = JointState()
        self.merged_map = None

    def execute_cb(self, goal):
        self.goal = goal
        self.aruco_id = self.goal.aruco_id
        self.tilt_angle = self.goal.tilt_angle
        self.fill_in_blindspot_with_second_scan = self.goal.fill_in_blindspot_with_second_scan
        self.fast_scan = self.goal.fast_scan
        self.publish_to_map = self.goal.publish_to_map
        self.scan_and_detect()

    def scan_and_detect(self):
        node = self
        self.aruco_tf = None
        self.predock_tf = None

        far_right_pan = -3.6 
        far_left_pan = 1.45 
        head_tilt = -0.8 
        num_pan_steps = 7 
        fast_scan = False
        capture_params = {
            'fast_scan': fast_scan,
            'head_settle_time': 0.5,
            'num_point_clouds_per_pan_ang': 10, # low numbers may not be effective for some surfaces and environments
            'time_between_point_clouds': 1.0/15.0 # point clouds at 15 Hz, so this should help obtain distinct clouds
        }

        head_scan = ma.HeadScan(voi_side_m=16.0)

        pose = {'joint_head_pan': far_right_pan, 'joint_head_tilt': head_tilt}
        node.move_to_pose(pose)
        
        pan_left = np.linspace(far_right_pan, far_left_pan, num_pan_steps)
        markers = []
        for pan_ang in pan_left:
            pose = {'joint_head_pan': pan_ang}
            head_scan.capture_point_clouds(node, pose, capture_params)
            
            for i in range(20):
                if self.markers:
                    markers = self.markers
                    break

            rospy.loginfo("Markers found: {}".format(markers))
            
            if markers != []:
                for marker in markers:
                    if marker.id == self.aruco_id:
                        self.aruco_found = True
                        self.aruco_name = marker.text
                        if self.publish_to_map:
                            try:
                                trans = self.tfBuffer.lookup_transform('map', self.aruco_name, rospy.Time())
                                self.aruco_tf = self.broadcast_tf(trans.transform, self.aruco_name, 'map')
                                rospy.loginfo("Pose published to tf")
                                if self.aruco_name == 'docking_station':
                                    # Transform the docking station frame such that x-axis points out of the aruco plane and 0.5 m in the front of the dock
                                    # This facilitates passing the goal pose as this predock frame so that the robot can back up into the dock
                                    saved_pose = Transform()
                                    saved_pose.translation.x = 0.0
                                    saved_pose.translation.y = -0.45
                                    saved_pose.translation.z = 0.47
                                    saved_pose.rotation.x = -0.382
                                    saved_pose.rotation.y = -0.352
                                    saved_pose.rotation.z = -0.604
                                    saved_pose.rotation.w = 0.604
                                    tran = self.broadcast_tf(saved_pose, 'predock_pose', 'docking_station')
                                    self.tf2_broadcaster.sendTransform(tran)
                                    trans = self.tfBuffer.lookup_transform('map', 'predock_pose', rospy.Time())
                                    # Bring predock_frame at base_link level
                                    trans.transform.translation.z = 0
                                    trans.header.stamp = rospy.Time.now()
                                    self.predock_tf = trans
                                    rospy.loginfo("Published predock pose")
                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                                rospy.loginfo("Could not publish pose to tf")
                                pass

            if not self.aruco_found:
                self.feedback.pan_angle = pan_ang
                self.server.publish_feedback(self.feedback)
            
        look_at_self = True
        if look_at_self:
            head_tilt = -1.2
            head_pan = 0.1
            pose = {'joint_head_pan': head_pan, 'joint_head_tilt': head_tilt}
            head_scan.capture_point_clouds(node, pose, capture_params)

        # record robot pose information and potentially useful transformations
        head_scan.robot_xy_pix, head_scan.robot_ang_rad, head_scan.timestamp = head_scan.max_height_im.get_robot_pose_in_image(node.tf2_buffer)

        head_scan.make_robot_mast_blind_spot_unobserved()
        head_scan.make_robot_footprint_unobserved()

        self.merged_map = head_scan
        self.result_cb(self.aruco_found, "after headscan")

    def result_cb(self, aruco_found, str=None):
        self.result.aruco_found = aruco_found
        if aruco_found:    
            rospy.loginfo("Aruco marker found")
            self.server.set_succeeded(self.result)
        else:
            rospy.loginfo("Could not find aruco marker {}".format(str))
            self.server.set_aborted(self.result)

    def broadcast_tf(self, trans, name, ref):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = ref
        t.child_frame_id = name
        t.transform = trans
        return t

    def aruco_callback(self, msg):
        self.markers = msg.markers

    def joint_state_callback(self, msg):
        pass

    def publish_map_point_cloud(self):
        if self.merged_map is not None:
            max_height_point_cloud = self.merged_map.max_height_im.to_point_cloud()
            self.point_cloud_pub.publish(max_height_point_cloud)

            # pub_voi = True
            # if pub_voi:
            #     marker = self.merged_map.max_height_im.voi.get_ros_marker(
            #         duration=1000.0)
            #     self.voi_marker_pub.publish(marker)
    
    def main(self):
        self.rate = 5.0
        rate = rospy.Rate(self.rate)
        
        self.point_cloud_pub = rospy.Publisher(
            '/funmap/point_cloud2', PointCloud2, queue_size=1)
        # self.voi_marker_pub = rospy.Publisher(
        #     '/funmap/voi_marker', Marker, queue_size=1)
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        self.map_to_odom_transform_mat = np.identity(4)
        while not rospy.is_shutdown():
            self.tf2_broadcaster.sendTransform(
                create_map_to_odom_transform(self.map_to_odom_transform_mat))
            self.publish_map_point_cloud()

            try:
                self.aruco_tf.header.stamp = rospy.Time.now()
                self.predock_tf.header.stamp = rospy.Time.now()
                self.tf2_broadcaster.sendTransform(self.aruco_tf)
                self.tf2_broadcaster.sendTransform(self.predock_tf)
            except AttributeError:
                pass
            rate.sleep()


if __name__ == '__main__':
    try:
        node = ArucoHeadScan()
        node.main()
        rospy.spin()
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')