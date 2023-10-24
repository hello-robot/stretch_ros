# ROS API for Stretch Core

*stretch_core* includes everything needed to interface with the sensors and actuators on Stretch. The API for the launch files, nodes, etc. are documented in this file.

## Nodes

### [stretch_driver](./nodes/stretch_driver)

This node communicates with the low-level Python library (stretch_body) to interface with the Stretch RE1/2.

#### Parameters

##### mode (string)

TODO

##### broadcast_odom_tf (boolean)

If set to true, stretch_driver will publish an odom to base_link TF.

##### rate (float)

TODO

##### timeout (float)

TODO

##### fail_out_of_range_goal (boolean)

TODO

#### Published Topics

##### /stretch/joint_states (sensor_msgs/JointState)

TODO

##### /odom (nav_msgs/Odometry)

TODO

##### /battery ([sensor_msgs/BatteryState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/BatteryState.html))

This topic publishes Stretch's battery and charge status. Charging status, the `power_supply_status` field, is estimated by looking at changes in voltage readings over time, where plugging-in causes the voltage to jump up (i.e. status becomes 'charging') and pulling the plug out is detected by a voltage dip (i.e. status becomes 'discharging'). Estimation of charging status is most reliable when the charger is in SUPPLY mode (see [docs here](https://docs.hello-robot.com/0.2/stretch-hardware-guides/docs/battery_maintenance_guide_re1/#charger) for how to change charging modes). Charging status is unknown at boot of this node. Consequently, the `current` field is positive at boot of this node, regardless of whether the robot is charging/discharging. After a charging state change, there is a ~10 second timeout where state changes won't be detected. Additionally, outlier voltage readings can slip past the filters and incorrectly indicate a charging state change (albeit rarely). Finally, voltage readings are affected by power draw (e.g. the onboard computer starts a computationally taxing program), which can lead to incorrect indications of charging state change. Stretch RE2s have a hardware switch in the charging port that can detect when a plug has been plugged in, regardless of whether the plug is providing any power. Therefore, this node combines the previous voltage-based estimate with readings from this hardware switch to make better charging state estimates on RE2s (effectively eliminating the false positive case where a computational load draws more power).

Since a battery is always present on a Stretch system, we instead misuse the `present` field to indicate whether a plug is plugged in to the charging port (regardless of whether it's providing power) on RE2 robots. This field is always false on RE1s. The unmeasured fields (e.g. charge in Ah) return a NaN, or 'not a number'.

##### /magnetometer_mobile_base (sensor_msgs/MagneticField)

TODO

##### /imu_mobile_base (sensor_msgs/Imu)

TODO

##### /imu_wrist (sensor_msgs/Imu)

TODO

##### /is_calibrated (std_msgs/Bool)

TODO

##### /mode (std_msgs/String)

TODO

#### Subscribed Topics

##### /stretch/cmd_vel (geometry_msgs/Twist)

#### Services

##### /switch_to_position_mode ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

TODO

##### /switch_to_navigation_mode ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

TODO

##### /stop_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

TODO

##### /runstop ([std_srvs/SetBool](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/SetBool.html))

TODO

##### /calibrate_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

This service will start Stretch's homing procedure, where every joint's zero is found. Robots with relative encoders (vs absolute encoders) need a homing procedure when they power on. For Stretch, it's a 30-second procedure that must occur everytime the robot wakes up before you may send motion commands to or read correct joint positions from Stretch's prismatic and multiturn revolute joints. When this service is triggered, the [mode topic](#mode-std_msgsstring) will reflect that the robot is in "calibration" mode, and after the homing procedure is complete, will switch back to whatever mode the robot was in before this service was triggered. While stretch_driver is in "calibration" mode, no commands to the [cmd_vel topic](#TODO) or the [follow joint trajectory action service](#TODO) will be accepted.

Other ways to home the robot include using the `stretch_robot_home.py` CLI tool from a terminal, or calling [`robot.home()`](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_stretch_body_api/#stretch_body.robot.Robot.home) from Stretch's Python API.

##### /stow_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

This service will start Stretch's stowing procedure, where the arm is stowed into the footprint of the mobile base. This service is more convenient than sending a [follow joint trajectory command](#TODO) since it knows what gripper is installed at the end of arm and stows these additional joints automatically.

Other ways to stow the robot include using the `stretch_robot_stow.py` CLI tool from a terminal, or calling [`robot.stow()`](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_stretch_body_api/#stretch_body.robot.Robot.stow) from Stretch's Python API.

#### Actions

##### /stretch_controller/follow_joint_trajectory ([control_msgs/FollowJointTrajectory](http://docs.ros.org/en/noetic/api/control_msgs/html/action/FollowJointTrajectory.html))

TODO

#### Transforms

See [stretch_driver.launch's TF docs](#TODO) to learn about the full Stretch TF tree. The stretch_driver node, which is part of stretch_driver.launch, is responsible for publishing the "odom" to "base_link" transform if the [broadcast_odom_tf parameter](#broadcastodomtf) is set to true. Odometry for the robot is calculated within the underlying Stretch Body Python SDK within the [update method in the Base class](https://github.com/hello-robot/stretch_body/blob/ea987c3d4f21a65ce4e85c6c92cd5d2efb832c41/body/stretch_body/base.py#L555-L651) by looking at the encoders for the left and right wheels. Note: Odometry calculated from wheel encoders is susceptible to drift for a variety of reasons (e.g. wheel slip, misalignment, loose belt tension, time dilation). A reasonable amount of drift is ~1cm per meter translated by the mobile base. A common remedy is to use a localization library to correct for the drift by integrating other sensors into the calculation of the odometry. For example, [AMCL](http://wiki.ros.org/amcl) is a particle filter ROS package for using Lidar scans + wheel odometry to correct for drift.

If you use [HelloNode](../hello_helpers/README.md#hellonode), you can get the odometry using:

```python
# roslaunch the stretch launch file beforehand

import rospy
import hello_helpers.hello_misc as hm
temp = hm.HelloNode.quick_create('temp')
t = temp.get_tf('odom', 'base_link')
print(t.transform.translation)
```

### [detect_aruco_markers](./nodes/detect_aruco_markers)

This node detects and estimates the pose of ArUco markers, including the markers on the robot's body.

TODO

### [d435i_configure](./nodes/d435i_configure)

This node lets you switch the visual preset of the D435i on the fly.

TODO
