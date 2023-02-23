#!/usr/bin/env python3

import time
import os
import sys
import glob
import math
import numbers

import rospy
import tf2_ros
import ros_numpy
import numpy as np
import cv2

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_ros
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, TriggerRequest


#######################
# initial code copied from stackoverflow.com on 10/20/2019
# https://stackoverflow.com/questions/45225474/find-nearest-white-pixel-to-a-given-pixel-location-opencv
def find_nearest_nonzero(img, target):
    nonzero = cv2.findNonZero(img)
    distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
    nearest_index = np.argmin(distances)
    nearest_x, nearest_y = nonzero[nearest_index][0]
    nearest_label = img[nearest_y, nearest_x]
    return nearest_x, nearest_y, nearest_label
#######################


def get_wrist_state(joint_states):
    telescoping_joints = ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3']
    wrist_velocity = 0.0
    wrist_position = 0.0
    wrist_effort = 0.0
    for joint_name in telescoping_joints:
        i = joint_states.name.index(joint_name)
        wrist_position += joint_states.position[i]
        wrist_velocity += joint_states.velocity[i]
        wrist_effort += joint_states.effort[i]

    wrist_effort = wrist_effort / len(telescoping_joints)
    return [wrist_position, wrist_velocity, wrist_effort]

def get_lift_state(joint_states):
    joint_name = 'joint_lift'
    i = joint_states.name.index(joint_name)
    lift_position = joint_states.position[i]
    lift_velocity = joint_states.velocity[i]
    lift_effort = joint_states.effort[i]
    return [lift_position, lift_velocity, lift_effort]

def get_left_finger_state(joint_states):
    joint_name =  'joint_gripper_finger_left'
    i = joint_states.name.index(joint_name)
    left_finger_position = joint_states.position[i]
    left_finger_velocity = joint_states.velocity[i]
    left_finger_effort = joint_states.effort[i]
    return [left_finger_position, left_finger_velocity, left_finger_effort]

class HelloNode:
    def __init__(self):
        self.joint_state = None
        self.point_cloud = None

    @classmethod
    def quick_create(cls, name, wait_for_first_pointcloud=False):
        i = cls()
        i.main(name, name, wait_for_first_pointcloud)
        return i

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def point_cloud_callback(self, point_cloud):
        self.point_cloud = point_cloud
    
    def move_to_pose(self, pose, return_before_done=False, custom_contact_thresholds=False, custom_full_goal=False):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(0.0)
        trajectory_goal = FollowJointTrajectoryGoal()
        trajectory_goal.goal_time_tolerance = rospy.Time(1.0)
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        trajectory_goal.trajectory.joint_names = list(pose.keys())
        trajectory_goal.trajectory.points = [point]

        # populate goal
        if custom_full_goal:
            pose_correct = all([len(joint_goal)==4 for joint_goal in pose.values()])
            if not pose_correct:
                rospy.logerr(f"{self.node_name}'s HelloNode.move_to_pose: Not sending trajectory due to improper pose. The 'custom_full_goal' option requires tuple with 4 values (pose_target, velocity, acceleration, contact_threshold_effort) for each joint name, but pose = {pose}")
                return
            point.positions = [joint_goal[0] for joint_goal in pose.values()]
            point.velocities = [joint_goal[1] for joint_goal in pose.values()]
            point.accelerations = [joint_goal[2] for joint_goal in pose.values()]
            point.effort = [joint_goal[3] for joint_goal in pose.values()]
        elif custom_contact_thresholds:
            pose_correct = all([len(joint_goal)==2 for joint_goal in pose.values()])
            if not pose_correct:
                rospy.logerr(f"{self.node_name}'s HelloNode.move_to_pose: Not sending trajectory due to improper pose. The 'custom_contact_thresholds' option requires tuple with 2 values (pose_target, contact_threshold_effort) for each joint name, but pose = {pose}")
                return
            point.positions = [joint_goal[0] for joint_goal in pose.values()]
            point.effort = [joint_goal[1] for joint_goal in pose.values()]
        else:
            pose_correct = all([isinstance(joint_position, numbers.Real) for joint_position in pose.values()])
            if not pose_correct:
                rospy.logerr(f"{self.node_name}'s HelloNode.move_to_pose: Not sending trajectory due to improper pose. The default option requires 1 value, pose_target as integer, for each joint name, but pose = {pose}")
                return
            point.positions = [joint_position for joint_position in pose.values()]

        # send goal
        self.trajectory_client.send_goal(trajectory_goal)
        if not return_before_done:
            self.trajectory_client.wait_for_result()
            rospy.logdebug(f"{self.node_name}'s HelloNode.move_to_pose: got result {self.trajectory_client.get_result()}")

    def get_robot_floor_pose_xya(self, floor_frame='odom'):
        # Returns the current estimated x, y position and angle of the
        # robot on the floor. This is typically called with respect to
        # the odom frame or the map frame. x and y are in meters and
        # the angle is in radians.
        
        # Navigation planning is performed with respect to a height of
        # 0.0, so the heights of transformed points are 0.0. The
        # simple method of handling the heights below assumes that the
        # frame is aligned such that the z axis is normal to the
        # floor, so that ignoring the z coordinate is approximately
        # equivalent to projecting a point onto the floor.
        
        # Query TF2 to obtain the current estimated transformation
        # from the robot's base_link frame to the frame.
        robot_to_odom_mat, timestamp = get_p1_to_p2_matrix('base_link', floor_frame, self.tf2_buffer)
        print('robot_to_odom_mat =', robot_to_odom_mat)
        print('timestamp =', timestamp)

        # Find the robot's current location in the frame.
        r0 = np.array([0.0, 0.0, 0.0, 1.0])
        print('r0 =', r0)
        r0 = np.matmul(robot_to_odom_mat, r0)[:2]

        # Find the current angle of the robot in the frame.
        r1 = np.array([1.0, 0.0, 0.0, 1.0])
        r1 = np.matmul(robot_to_odom_mat, r1)[:2]
        robot_forward = r1 - r0
        r_ang = np.arctan2(robot_forward[1], robot_forward[0])

        return [r0[0], r0[1], r_ang], timestamp

    def get_tf(self, from_frame, to_frame):
        """Get current transform between 2 frames. Blocking.
        """
        rate = rospy.Rate(10.0)
        while True:
            try:
                return self.tf2_buffer.lookup_transform(from_frame, to_frame, rospy.Time())
            except:
                continue
            rate.sleep()

    def main(self, node_name, node_topic_namespace, wait_for_first_pointcloud=True):
        rospy.init_node(node_name)
        self.node_name = rospy.get_name()
        rospy.loginfo("{0} started".format(self.node_name))

        self.trajectory_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        server_reached = self.trajectory_client.wait_for_server(timeout=rospy.Duration(60.0))
        if not server_reached:
            rospy.signal_shutdown('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()
        
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        
        self.point_cloud_subscriber = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.point_cloud_callback)
        self.point_cloud_pub = rospy.Publisher('/' + node_topic_namespace + '/point_cloud2', PointCloud2, queue_size=1)

        rospy.wait_for_service('/stop_the_robot')
        rospy.loginfo('Node ' + self.node_name + ' connected to /stop_the_robot service.')
        self.stop_the_robot_service = rospy.ServiceProxy('/stop_the_robot', Trigger)
        
        if wait_for_first_pointcloud:
            # Do not start until a point cloud has been received
            point_cloud_msg = self.point_cloud
            print('Node ' + node_name + ' waiting to receive first point cloud.')
            while point_cloud_msg is None:
                rospy.sleep(0.1)
                point_cloud_msg = self.point_cloud
            print('Node ' + node_name + ' received first point cloud, so continuing.')


def create_time_string():
    t = time.localtime()
    time_string = str(t.tm_year) + str(t.tm_mon).zfill(2) + str(t.tm_mday).zfill(2) + str(t.tm_hour).zfill(2) + str(t.tm_min).zfill(2) + str(t.tm_sec).zfill(2)
    return time_string


def get_recent_filenames(filename_without_time_suffix, filename_extension, remove_extension=False): 
    filenames = glob.glob(filename_without_time_suffix + '_*[0-9]' + '.' + filename_extension)
    filenames.sort()
    if remove_extension:
        return [os.path.splitext(f)[0] for f in filenames]
    return filenames


def get_most_recent_filename(filename_without_time_suffix, filename_extension, remove_extension=False):
    filenames = get_recent_filenames(filename_without_time_suffix, filename_extension, remove_extension=remove_extension) 
    most_recent_filename = filenames[-1]
    return most_recent_filename


def angle_diff_deg(target_deg, current_deg):
    # I've written this type of function many times before, and it's
    # always been annoying and tricky. This time, I looked on the web:
    # https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    diff_deg = target_deg - current_deg
    diff_deg = ((diff_deg + 180.0) % 360.0) - 180.0
    return diff_deg


def angle_diff_rad(target_rad, current_rad):
    # I've written this type of function many times before, and it's
    # always been annoying and tricky. This time, I looked on the web:
    # https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
    diff_rad = target_rad - current_rad
    diff_rad = ((diff_rad + math.pi) % (2.0 * math.pi)) - math.pi
    return diff_rad


def get_p1_to_p2_matrix(p1_frame_id, p2_frame_id, tf2_buffer, lookup_time=None, timeout_s=None):
    # If the necessary TF2 transform is successfully looked up, this
    # returns a 4x4 affine transformation matrix that transforms
    # points in the p1_frame_id frame to points in the p2_frame_id.
    try:
        if lookup_time is None:
            lookup_time = rospy.Time(0) # return most recent transform
        if timeout_s is None:
            timeout_ros = rospy.Duration(0.1)
        else:
            timeout_ros = rospy.Duration(timeout_s)
        stamped_transform =  tf2_buffer.lookup_transform(p2_frame_id, p1_frame_id, lookup_time, timeout_ros)

        # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TransformStamped.html

        p1_to_p2_mat = ros_numpy.numpify(stamped_transform.transform)
        return p1_to_p2_mat, stamped_transform.header.stamp
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print('WARNING: get_p1_to_p2_matrix failed to lookup transform from p1_frame_id =', p1_frame_id, ' to p2_frame_id =', p2_frame_id)
        print('         exception =', e)
        return None, None

def bound_ros_command(bounds, ros_pos, fail_out_of_range_goal, clip_ros_tolerance=1e-3):
    """Clip the command with clip_ros_tolerance, instead of
    invalidating it, if it is close enough to the valid ranges.
    """
    if ros_pos < bounds[0]:
        if fail_out_of_range_goal:
            return bounds[0] if (bounds[0] - ros_pos) < clip_ros_tolerance else None
        else:
            return bounds[0]

    if ros_pos > bounds[1]:
        if fail_out_of_range_goal:
            return bounds[1] if (ros_pos - bounds[1]) < clip_ros_tolerance else None
        else:
            return bounds[1]

    return ros_pos

def compare_versions(v1, v2):
    """
    Compare two strings of versions.
    Returns 1  if v1>v2
            -1 if v1<v2
            0  if v1==v2
    """
    v1 = [int(v) for v in v1.split('.')]
    v2 = [int(v) for v in v2.split('.')]
    while len(v1) < len(v2):
        v1.append(0)
    while len(v2) < len(v1):
        v2.append(0)
    for i in range(len(v1)):
        if v1[i] < v2[i]:
            return -1
        elif v1[i] > v2[i]:
            return 1
    return 0