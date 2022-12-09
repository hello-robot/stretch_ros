![](../images/banner.png)

## Overview

*hello_helpers* mostly consists of the hello_helpers Python module. This module provides various Python files used across stretch_ros that have not attained sufficient status to stand on their own.

## Capabilities

*fit_plane.py* : Fits planes to 3D data.

*hello_misc.py* : Various functions, including a helpful Python object with which to create ROS nodes. 

*hello_ros_viz.py* : Various helper functions for vizualizations using RViz.

## Typical Usage

```
import hello_helpers.fit_plane as fp
```
```
import hello_helpers.hello_misc as hm
```
```
import hello_helpers.hello_ros_viz as hr
```

# API

## Classes

### [HelloNode](./src/hello_helpers/hello_misc.py)

This class is a convenience class for creating a ROS node for Stretch. The most common way to use this class is to extend it. In your extending class, the main funcion would call `HelloNode`'s main function. This would look like:

```
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

```
# roslaunch the stretch_driver launch file beforehand

import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
temp.move_to_pose({'joint_lift': 0.4})
```

#### Methods

##### `move_to_pose(pose, return_before_done=False, custom_full_goal=False, custom_contact_thresholds=False)`

This method takes in a dictionary that describes a desired pose for the robot and moves the executes it.

#### Published Topics

##### /battery ([sensor_msgs/BatteryState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/BatteryState.html))

This topic publishes Stretch's battery and charge status. Charging status, the `power_supply_status` field, is estimated by looking at changes in voltage readings over time, where plugging-in causes the voltage to jump up (i.e. status becomes 'charging') and pulling the plug out is detected by a voltage dip (i.e. status becomes 'discharging'). Estimation of charging status is most reliable when the charger is in SUPPLY mode (see [docs here](https://docs.hello-robot.com/0.2/stretch-hardware-guides/docs/battery_maintenance_guide_re1/#charger) for how to change charging modes). Charging status is unknown at boot of this node. Consequently, the `current` field is positive at boot of this node, regardless of whether the robot is charging/discharging. After a charging state change, there is a ~10 second timeout where state changes won't be detected. Additionally, outlier voltage readings can slip past the filters and incorrectly indicate a charging state change (albeit rarely). Finally, voltage readings are affected by power draw (e.g. the onboard computer starts a computationally taxing program), which can lead to incorrect indications of charging state change. Stretch RE2s have a hardware switch in the charging port that can detect when a plug has been plugged in, regardless of whether the plug is providing any power. Therefore, this node combines the previous voltage-based estimate with readings from this hardware switch to make better charging state estimates on RE2s (effectively eliminating the false positive case where a computational load draws more power).

Since a battery is always present on a Stretch system, we instead misuse the `present` field to indicate whether a plug is plugged in to the charging port (regardless of whether it's providing power) on RE2 robots. This field is always false on RE1s. The unmeasured fields (e.g. charge in Ah) return a NaN, or 'not a number'.

#### Published Services

##### /calibrate_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

This service will start Stretch's homing procedure, where every joint's zero is found. Robots with relative encoders (vs absolute encoders) need a homing procedure when they power on. For Stretch, it's a 30-second procedure that must occur everytime the robot wakes up before you may send motion commands to or read correct joint positions from Stretch's prismatic and multiturn revolute joints. When this service is triggered, the [mode topic](#mode-std_msgsstring) will reflect that the robot is in "calibration" mode, and after the homing procedure is complete, will switch back to whatever mode the robot was in before this service was triggered. While stretch_driver is in "calibration" mode, no commands to the [cmd_vel topic](#TODO) or the [follow joint trajectory action service](#TODO) will be accepted.

Other ways to home the robot include using the `stretch_robot_home.py` CLI tool from a terminal, or calling [`robot.home()`](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_stretch_body_api/#stretch_body.robot.Robot.home) from Stretch's Python API.

##### /stow_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

This service will start Stretch's stowing procedure, where the arm is stowed into the footprint of the mobile base. This service is more convenient than sending a [follow joint trajectory command](#TODO) since it knows what gripper is installed at the end of arm and stows these additional joints automatically.

Other ways to stow the robot include using the `stretch_robot_stow.py` CLI tool from a terminal, or calling [`robot.stow()`](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_stretch_body_api/#stretch_body.robot.Robot.stow) from Stretch's Python API.

## License

For license information, please see the LICENSE files. 
