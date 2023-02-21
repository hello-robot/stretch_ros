![](../images/banner.png)

## Overview

*hello_helpers* mostly consists of the hello_helpers Python module. This module provides various Python files used across stretch_ros that have not attained sufficient status to stand on their own.

## Capabilities

*fit_plane.py* : Fits planes to 3D data.

*hello_misc.py* : Various functions, including a helpful Python object with which to create ROS nodes. 

*hello_ros_viz.py* : Various helper functions for vizualizations using RViz.

## Typical Usage

```python
import hello_helpers.fit_plane as fp
```
```python
import hello_helpers.hello_misc as hm
```
```python
import hello_helpers.hello_ros_viz as hr
```

# API

## Classes

### [HelloNode](./src/hello_helpers/hello_misc.py)

This class is a convenience class for creating a ROS node for Stretch. The most common way to use this class is to extend it. In your extending class, the main funcion would call `HelloNode`'s main function. This would look like:

```python
import hello_helpers.hello_misc as hm

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(self, 'my_node', 'my_node', wait_for_first_pointcloud=False)
        # my_node's main logic goes here

node = MyNode()
node.main()
```

There is also a one-liner class method for instantiating a `HelloNode` for easy prototyping. One example where this is handy is sending pose commands from iPython:

```python
# roslaunch the stretch launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.move_to_pose({'joint_lift': 0.4})
```

#### Methods

##### `move_to_pose(pose, return_before_done=False, custom_contact_thresholds=False, custom_full_goal=False)`

This method takes in a dictionary that describes a desired pose for the robot and communicates with [stretch_driver](../stretch_core/README.md#stretchdrivernodesstretchdriver) to execute it. The basic format of this dictionary is string/number key/value pairs, where the keys are joint names and the values are desired position goals. For example, `{'joint_lift': 0.5}` would put the lift at 0.5m in its joint range. A full list of command-able joints is published to the `/stretch/joint_states` topic. Used within a node extending `HelloNode`, calling this method would look like:

```python
self.move_to_pose({'joint_lift': 0.5})
```

Internally, this dictionary is converted into a [FollowJointTrajectoryGoal](http://docs.ros.org/en/diamondback/api/control_msgs/html/msg/FollowJointTrajectoryGoal.html) that is sent to a [FollowJointTrajectory action](http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html) server in stretch_driver. This method waits by default for the server to report that the goal has completed executing. However, you can return before the goal has completed by setting the `return_before_done` argument to True. This can be useful for preempting goals.

There are two additional arguments that enable you to customize how the pose is executed. If you set `custom_contact_thresholds` to True, this method expects a different format dictionary: string/tuple key/value pairs, where the keys are still joint names, but the values are `(position_goal, effort_threshold)`. The addition of a effort threshold enables you to detect when a joint has made contact with something in the environment, which is useful for manipulation or safe movements. For example, `{'joint_arm': (0.5, 20)}` commands the telescoping arm fully out (the arm is nearly fully extended at 0.5 meters) but with a low enough effort threshold (20% of the arm motor's max effort) that the motor will stop when the end of arm has made contact with something. Again, in a node, this would look like:

```python
self.move_to_pose({'joint_arm': (0.5, 40)}, custom_contact_thresholds=True)
```

If you set `custom_full_goal` to True, the dictionary format is string/tuple key/value pairs, where keys are joint names again, but values are `(position_goal, velocity, acceleration, effort_threshold)`. The velocity and acceleration components allow you to customize the trajectory profile the joint follows while moving to the goal position. In the following example, the arm telescopes out slowly until contact is detected:

```python
self.move_to_pose({'joint_arm': (0.5, 0.01, 0.01, 40)}, custom_full_goal=True)
```

##### `get_tf(from_frame, to_frame)`

Use this method to get the transform ([geometry_msgs/TransformStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TransformStamped.html)) between two frames. This method is blocking. For example, this method can do forward kinematics from the base_link to the link between the gripper fingers, link_grasp_center, using:

```python
# roslaunch the stretch launch file beforehand

import rospy
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
t = temp.get_tf('base_link', 'link_grasp_center')
print(t.transform.translation)
```

##### `get_robot_floor_pose_xya(floor_frame='odom')`

Returns the current estimated x, y position and angle of the robot on the floor. This is typically called with respect to the odom frame or the map frame. x and y are in meters and the angle is in radians.

##### `main(node_name, node_topic_namespace, wait_for_first_pointcloud=True)`

When extending the `HelloNode` class, call this method at the very beginning of your `main()` method. This method handles setting up a few ROS components, including registering the node with the ROS server, creating a TF listener, creating a [FollowJointTrajectory](http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html) client for the [`move_to_pose()`](#movetoposepose-returnbeforedonefalse-customcontactthresholdsfalse-customfullgoalfalse) method, subscribing to depth camera point cloud topic, and connecting to the quick-stop service.

Since it takes up to 30 seconds for the head camera to start streaming data, the `wait_for_first_pointcloud` argument will get the node to wait until it has seen camera data, which is helpful if your node is processing camera data.

##### `quick_create(name, wait_for_first_pointcloud=False)`

A class level method for quick testing. This allows you to avoid having to extend `HelloNode` to use it.

```python
# roslaunch the stretch launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.move_to_pose({'joint_lift': 0.4})
```

#### Subscribed Topics

##### /camera/depth/color/points ([sensor_msgs/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))

Provides a point cloud as currently seen by the Realsense depth camera in Stretch's head. Accessible from the `self.point_cloud` attribute.

```python
# roslaunch the stretch launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp', wait_for_first_pointcloud=True)
print(temp.point_cloud)
```

#### Subscribed Services

##### /stop_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

Provides a service to quickly stop any motion currently executing on the robot.

```python
# roslaunch the stretch launch file beforehand

from std_srvs.srv import TriggerRequest
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.stop_the_robot_service(TriggerRequest())
```

## License

For license information, please see the LICENSE files. 
