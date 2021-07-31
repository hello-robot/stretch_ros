#! /usr/bin/env python3

from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetWorldProperties
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import Empty
import rospy
import time

rospy.init_node('ground_truth_odometry_publisher')
odom_pub=rospy.Publisher('ground_truth', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

odom=Odometry()
header = Header()
header.frame_id='/ground_truth'
model = GetModelStateRequest()
model.model_name='robot'
models = []
r = rospy.Rate(20)

pause_timeout = time.time() + 4.0
while time.time() < pause_timeout:
    rospy.logwarn("Waiting %.2f seconds to unpause physics", pause_timeout - time.time())
    time.sleep(1.0)
unpause_physics()

while not rospy.is_shutdown():
    if model.model_name not in models:
        models = get_world_properties().model_names
        rospy.logwarn("Waiting for %s to spawn to publish ground truth odometry", model.model_name)
    else:
        result = get_model_srv(model)
        odom.pose.pose = result.pose
        odom.twist.twist = result.twist
        header.stamp = rospy.Time.now()
        odom.header = header
        odom_pub.publish(odom)
    r.sleep()
