![](../images/banner.png)

# Overview

*stretch_core* provides the core ROS interfaces to the Stretch RE1 mobile manipulator from Hello Robot Inc. It includes the following nodes:

*stretch_driver* : node that communicates with the low-level Python library (stretch_body) to interface with the Stretch RE1

*detect_aruco_markers* : node that detects and estimates the pose of ArUco markers, including the markers on the robot's body

*d435i_* : various nodes to help use the Stretch RE1's 3D camera

*keyboard_teleop* : node that provides a keyboard interface to control the robot's joints

# API

## Launch Files

### [stretch.launch](./launch/stretch.launch)

This launch file is commonly used by other ROS packages to bring up everything necessary to work with Stretch from ROS. It includes the drivers for:

 - Sending commands to the robot and reading joint state
 - Reading color and depth imagery from the head camera
 - Reading laser scans from the base lidar
 - Calculating fused lidar / wheel odometry
 - Reading audio from the microphone array + sending audio to the speakers

#### Arguments

##### lidar_odom

Type: Boolean. Controls whether the odom TF is estimated with lidar odometry fused, or just wheel odometry.

##### respeaker

Type: Boolean. Controls whether the respeaker microphone array/speaker drivers are launched.

##### rviz

Type: Boolean. Controls whether Rviz is shown.

### [stretch_driver.launch](./launch/stretch_driver.launch)

This launch file is commonly used by other ROS packages to bring up everything necessary to send joint commands to the robot and read joint state from the robot. It launches the [stretch_driver](#stretch_driver) node (documented below) which plays a critical role in interfacing with the underlying hardware. Supporting nodes like robot_state_publisher, joint_state_publisher, and diagnostics_aggregator are also launched to enable ROS features like TF lookups and diagnostic GUIs.

#### Arguments

##### calibrated_controller_yaml_file

This ROS arg is required and defaults to "$(rospack find stretch_core)/config/controller_calibration_head.yaml", a YAML file that is generated when the robot is calibrated for joint offsets/backlashes through the [Stretch Calibration](../stretch_calibration/) package. The *stretch_driver* node loads in these calibrated offset/backlash parameters, and accounts for them when reporting joint state.

### [keyboard_teleop.launch](./launch/keyboard_teleop.launch)

This launch file enables easy keyboard teleoperation of Stretch's joints. It internally launches the [stretch_driver.launch](#stretch_driverlaunch) launch file, as well as the [keyboard_teleop](./nodes/keyboard_teleop) node.

### [rplidar.launch](./launch/rplidar.launch)

This launch file brings up the drivers for the RPLidar A1, a 2D planar lidar that sits on the base of the robot. More info on the RPLidar's ROS drivers can be found in Slamtec's [Github repo](https://github.com/Slamtec/rplidar_ros). This launch file also includes a node to filter out shadows from raw laser scans.

### [stretch_scan_matcher.launch](./launch/stretch_scan_matcher.launch)

If you have [rplidar.launch](#rplidarlaunch) and [stretch_driver.launch](#stretch_driverlaunch) running, you can use this launch file to get a better estimate of the robot's location as it moves. This is done using the [laser_scan_matcher](http://wiki.ros.org/laser_scan_matcher) package, which fuses laser scans with wheel odometry and publishes the estimated pose of the robot w.r.t to a "odom" frame.

### [d435i_high_resolution.launch](./launch/d435i_high_resolution.launch)

This launch file brings up the drivers for the Intel D435i Realsense depth camera. It builds on the [d435i_basic.launch](./launch/d435i_basic.launch) to provide high definition (HD) resolution RGBD imagery under topics in the "/camera" namespace. The camera is launched with the high accurary visual preset, which is defined in a JSON file at "$(rospack find stretch_core)/config/HighAccuraryPreset.json". More details can be found in Intel's realsense-ros [github repo](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy#usage-instructions).

### [d435i_low_resolution.launch](./launch/d435i_low_resolution.launch)

This launch file is similar to the [d435i_high_resolution.launch](#d435i_high_resolutionlaunch) launch file, but provides the RGBD imagery at standard definition (SD) resolution instead of HD.

### [stretch_aruco.launch](./launch/stretch_aruco.launch)

If you have one of the [D435i](#d435i_high_resolutionlaunch) launch files and [stretch_driver.launch](#stretch_driverlaunch) launch file running, you can use this launch file to enable Aruco detection from Stretch's head camera. The [detect_aruco_markers](#detect_aruco_markers) node (documented below) plays a critical role in enabling this functionality. The list of known markers lives in a YAML file at "$(rospack find stretch_core)/config/stretch_marker_dict.yaml". Learn how to use this launch file by following the [Aruco Marker Detection](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/aruco_marker_detection/) tutorial.

## Nodes

### [stretch_driver](./nodes/stretch_driver)

#### Parameters

##### broadcast_odom_tf

If set to true, stretch_driver will publish an odom to base_link TF.

#### Published Topics

##### /mode ([std_msgs/String](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html))

This topic publishes which mode the driver is in at 15hz. stretch_driver has a few modes that change how the robot is controlled. The modes are:

 - "position": The default and simplest mode. In this mode, you can control every joint on the robot using position commands. For example, the telescoping arm (whose range is from zero fully retracted to ~52 centimeters fully extended) would move to 25cm out when you send it a [0.25m position cmd](../hello_helpers/README.md#movetoposepose-returnbeforedonefalse-customcontactthresholdsfalse-customfullgoalfalse). Two kinds of position commands are available for the mobile base, translation and rotation, with joint names "translate_mobile_base" and "rotate_mobile_base", and these commands have no limits since the wheels can spin continuously.
   - Position commands are tracked in the firmware by a trapezoidal motion profile, and specifing the optional velocity and acceleration in [JointTrajectoryPoint](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html) changes the shape of the trapezoid. More details about the motion profile [in the tutorial](https://docs.hello-robot.com/0.2/stretch-tutorials/ros1/follow_joint_trajectory/).
   - Position commands can be contact sensitive, which is helpful for manipulating objects in the world. For example, I could [open a cabinet](https://youtu.be/SXgj9be3PdM) by reaching out with the telescoping arm and detecting contact with the door. In order to specify contact thresholds for a position command, the optional effort in [JointTrajectoryPoint](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html) is misused to mean a threshold for the effort the robot will apply while executing the position command.
   - Position commands can be preemptable, so you can issue a new position command before the previous one has finished and the robot will smoothly move to execute the latest command. This feature is helpful for scripts that use visual servo-ing or allow a user to teleop the robot.
   - The driver can be switched into "position" mode at any time using the [switch_to_position_mode service](#TODO).
 - "navigation": In this mode, every joint behaves identically to "position" mode except for the mobile base. The mobile base responds to velocity commands at a topic instead of position commands via the "translate_mobile_base" and "rotate_mobile_base" joints in the action server. You would publish [geometry_msgs/Twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) messages to the [/stretch/cmd_vel topic](#TODO). Since Twist messages are generalized to robots that can move with velocity in any direction, only the `Twist.linear.x` (translational velocity) and `Twist.angular.z` (rotational velocity) fields apply for differential drive mobile bases.
   - Velocity control of the base is a common way to move mobile robots around. For example, the [Navigation Stack](https://github.com/hello-robot/stretch_ros/blob/noetic/stretch_navigation/README.md) is a piece of software that uses this mode to move the robot to different points in a map.
   - This mode has a few safety features to prevent Stretch from "running away" at the last commanded velocity if the node that was sending commands were to crash for whatever reason. The first is a 0.5 second timeout within the stretch_driver node, which means that if the driver doesn't receive a new Twist command within 0.5s, the base will be commanded to stop smoothly. The second is a 1 second timeout within the firmware of the wheels, which means that even if the ROS layer were to crash, the robot will still be commanded to stop abruptly within 1 second at the lowest layer. This low-level feature was added relatively recently to the firmware, so be sure to [update your firmware](https://github.com/hello-robot/stretch_firmware/blob/master/docs/tutorial_updating_firmware.md) to the latest version.
   - The driver can be switched into "navigation" mode at any time using the [switch_to_navigation_mode service](#TODO).
 - "trajectory": **Not available in ROS1, check out [Stretch ROS2](https://github.com/hello-robot/stretch_ros2)**. In this mode, every joint follows a trajectory that is modeled as a spline. The key benefit of this mode is control over the timing at which the robot achieves positions (a.k.a waypoints), enabling smooth and coordinated motion through a preplanned trajectory. [More details here](https://forum.hello-robot.com/t/creating-smooth-motion-using-trajectories/671).
 - "homing": This is the mode reported by the driver when a homing sequence has been triggered (e.g. through the [home_the_robot service](#hometherobot-stdsrvstriggerhttpsdocsrosorgennoeticapistdsrvshtmlsrvtriggerhtml)). While this mode is active, no other commands will be accepted by the driver. After the robot has completed its 30 second homing sequence, it will return to the mode it was in before.
 - "stowing": This is the mode reported by the driver when a stowing sequence has been triggered (e.g. through the [stow_the_robot service](#stowtherobot-stdsrvstriggerhttpsdocsrosorgennoeticapistdsrvshtmlsrvtriggerhtml)). While this mode is active, no other commands will be accepted by the driver. After the robot has completed its stowing sequence, it will return to the mode it was in before.
 - "runstopped": This is the mode reported by the driver when the robot is in runstop, either through the user pressing the [glowing white button](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/safety_guide/#runstop) in Stretch's head or through the [runstop service](#runstop-stdsrvssetboolhttpsdocsrosorgennoeticapistdsrvshtmlsrvsetboolhtml). While this mode is active, no other commands will be accepted by the driver. After the robot has been taken out of runstop, it will return to the mode it was in before, or "position" mode if the driver was launched while the robot was runstopped.
 - "manipulation": **Deprecated**. This mode was previously available in ROS1 Melodic, but was removed when the driver was ported to ROS1 Noetic. The mode supported a virtual prismatic joint for the mobile base, allowing you to treat the mobile base as a joint that could move forwards/backwards 0.5m. It was removed because the mode had little utility for most users of the driver.

##### /battery ([sensor_msgs/BatteryState](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/BatteryState.html))

This topic publishes Stretch's battery and charge status. Charging status, the `power_supply_status` field, is estimated by looking at changes in voltage readings over time, where plugging-in causes the voltage to jump up (i.e. status becomes 'charging') and pulling the plug out is detected by a voltage dip (i.e. status becomes 'discharging'). Estimation of charging status is most reliable when the charger is in SUPPLY mode (see [docs here](https://docs.hello-robot.com/0.2/stretch-hardware-guides/docs/battery_maintenance_guide_re1/#charger) for how to change charging modes). Charging status is unknown at boot of this node. Consequently, the `current` field is positive at boot of this node, regardless of whether the robot is charging/discharging. After a charging state change, there is a ~10 second timeout where state changes won't be detected. Additionally, outlier voltage readings can slip past the filters and incorrectly indicate a charging state change (albeit rarely). Finally, voltage readings are affected by power draw (e.g. the onboard computer starts a computationally taxing program), which can lead to incorrect indications of charging state change. Stretch RE2s have a hardware switch in the charging port that can detect when a plug has been plugged in, regardless of whether the plug is providing any power. Therefore, this node combines the previous voltage-based estimate with readings from this hardware switch to make better charging state estimates on RE2s (effectively eliminating the false positive case where a computational load draws more power).

Since a battery is always present on a Stretch system, we instead misuse the `present` field to indicate whether a plug is plugged in to the charging port (regardless of whether it's providing power) on RE2 robots. This field is always false on RE1s. The unmeasured fields (e.g. charge in Ah) return a NaN, or 'not a number'.

##### /is_homed ([std_msgs/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

This topic publishes whether Stretch's encoders has been homed. If the robot isn't homed, joint states will be incorrect and motion commands won't be accepted by the driver. The [home_the_robot service](#hometherobot-stdsrvstriggerhttpsdocsrosorgennoeticapistdsrvshtmlsrvtriggerhtml) can be used to home Stretch.

##### /is_runstopped ([std_msgs/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

This topic publishes whether Stretch is runstopped. When the robot is runstopped (typically by [pressing glowing white button](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/safety_guide/#runstop) in Stretch's head), motion for all joints is stopped and new motion commands aren't accepted. This is a safety feature built into the firmware for the four primary actuators. It's also possible to runstop the robot programmatically using the [runstop service](#runstop-stdsrvssetboolhttpsdocsrosorgennoeticapistdsrvshtmlsrvsetboolhtml)

##### /is_calibrated ([std_msgs/Bool](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html))

**Deprecated:** This topic has been renamed to [is_homed](#ishomed-stdmsgsboolhttpsdocsrosorgennoeticapistdmsgshtmlmsgboolhtml) because we often use the terms "calibrated" or "calibration" in context of [URDF calibrations](../stretch_calibration/README.md), whereas this topic returns whether the robot's encoders are homed.

#### Published Services

##### /home_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

This service will start Stretch's homing procedure, where every joint's zero is found. Robots with relative encoders (vs absolute encoders) need a homing procedure when they power on. For Stretch, it's a 30-second procedure that must occur everytime the robot wakes up before you may send motion commands to or read correct joint positions from Stretch's prismatic and multiturn revolute joints. When this service is triggered, the [mode topic](#mode-stdmsgsstringhttpsdocsrosorgennoeticapistdmsgshtmlmsgstringhtml) will reflect that the robot is in "homing" mode, and after the homing procedure is complete, will switch back to whatever mode the robot was in before this service was triggered. While stretch_driver is in "homing" mode, no commands to the [cmd_vel topic](#TODO) or the [follow joint trajectory action service](#TODO) will be accepted.

Other ways to home the robot include using the `stretch_robot_home.py` CLI tool from a terminal, or calling [`robot.home()`](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_stretch_body_api/#stretch_body.robot.Robot.home) from Stretch's Python API.

Runstopping the robot while this service is running will yield undefined behavior and likely leave the driver in a bad state.

##### /stow_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

This service will start Stretch's stowing procedure, where the arm is stowed into the footprint of the mobile base. This service is more convenient than sending a [follow joint trajectory command](#TODO) since it knows what gripper is installed at the end of arm and stows these additional joints automatically. When this service is triggered, the [mode topic](#mode-stdmsgsstringhttpsdocsrosorgennoeticapistdmsgshtmlmsgstringhtml) will reflect that the robot is in "stowing" mode, and after the homing procedure is complete, will switch back to whatever mode the robot was in before this service was triggered. While stretch_driver is in "stowing" mode, no commands to the [cmd_vel topic](#TODO) or the [follow joint trajectory action service](#TODO) will be accepted.

Other ways to stow the robot include using the `stretch_robot_stow.py` CLI tool from a terminal, or calling [`robot.stow()`](https://docs.hello-robot.com/0.2/stretch-tutorials/stretch_body/tutorial_stretch_body_api/#stretch_body.robot.Robot.stow) from Stretch's Python API.

Runstopping the robot while this service is running will yield undefined behavior and likely leave the driver in a bad state.

##### /stop_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

This service immediately stops any currently active motion.

##### /runstop ([std_srvs/SetBool](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/SetBool.html))

This service can put Stretch into runstop or take Stretch out of runstop. It's common to put the robot into/out of runstop by pressing the [glowing white button](https://docs.hello-robot.com/0.2/stretch-tutorials/getting_started/safety_guide/#runstop) in Stretch's head (at which point the robot will beep and the button will be blinking to indicate that it's runstopped), and holding the button down for two seconds to take it out of runstop (the button will return to non-blinking). This service acts as a programmatic way to achieve the same effect. When this service is triggered, the [mode topic](#mode-stdmsgsstringhttpsdocsrosorgennoeticapistdmsgshtmlmsgstringhtml) will reflect that the robot is in "runstopped" mode, and after the robot is taken out of runstop, the driver will switch back to whatever mode the robot was in before this service was triggered. While stretch_driver is in "runstopped" mode, no commands to the [cmd_vel topic](#TODO) or the [follow joint trajectory action service](#TODO) will be accepted.

##### /calibrate_the_robot ([std_srvs/Trigger](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Trigger.html))

**Deprecated:** This service has been renamed to [home_the_robot](#hometherobot-stdsrvstriggerhttpsdocsrosorgennoeticapistdsrvshtmlsrvtriggerhtml) because we often use the terms "calibrate" or "calibration" in context of [URDF calibrations](../stretch_calibration/README.md), whereas this service homes the robot's encoders.

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

# Testing

The primary testing framework being used within *stretch_ros* is pytest. Pytest is an open source testing framework that scales well and takes a functional approach resulting in minimal boiler plate code. First we should ensure that pytest is installed and up to date:


```bash
>>$ pip3 install -U pytest
>>$ pytest --version
pytest 6.2.4
```
Testing can be split into four different categories:

1. Python library unit tests (pytest)
2. ROS node unit tests (pytest + rostest)
3. ROS node integration tests (pytest + rostest)
4. ROS functional tests (pytest + rostest)

Tests suites are organized using the rostest framework which is an extension of standard ROS launch files. Test suites can be created simply by adding the following lines to the desired test file:

Python Library Tests:

```xml
<launch>
  <param name="test_module" value="../src"/>
  <test test-name="test_lib" pkg="my_pkg" type="pytest_runner.py" />
</launch>
```

ROS Node Unit/Integration Tests:

```xml
<launch>
  <node pkg="my_pkg" type="publisher" name="publisher" />
  <param name="test_module" value="listener"/>
  <test test-name="test_listener" pkg="my_pkg" type="pytest_runner.py" />
</launch>
```

It is also important that test suites are created in a directory that has access to the *pytest_runner.py* file. This script converts the output file parameter to the pytest supported format. Additionally, it passes the test directory to pytest.

Below are some optional plugins that help make test reports more readable and with decoupling of test dependencies:

 * Pytest Clarity: https://github.com/darrenburns/pytest-clarity
 * Pytest Randomly: https://github.com/pytest-dev/pytest-randomly

Before running any tests we should run the *stretch_robot_home.py* script at least once after startup and launch test suites that require nodes to be running via the roslaunch command as follows (we should not launch any test suites if we are using the *catkin_tools* package to build and run tests):

```bash
>>$ stretch_robot_home.py
```

```bash
>>$ roslaunch <package_name> <test_suite_name>
```

In order to run tests the following commands can be typed into the command line:

```bash
>>$ pytest -vv
```
A test session will bootup that reports the root directory and all the plugins currently running. Within the test session pytest will search for all tests in the current directory and all subdirectories.

We can also run individual tests by specify the test name after the option -k as follows:

```bash
>>$ pytest -vv -k 'test_name'
```
In order to run all tests in all packages, we should call the *pytest* command inside of our *catkin_ws*.

Note, we can also run tests using the *catkin_tools* package. Thus we can assume we will be using *catkin build* instead of *catkin_make*. To install *catkin_tools* first ensure that we have the ROS repositories that contain *.deb* and *catkin_tools*:

```bash
>>$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
>>$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

Next we can install *catkin_tools* by running:

```bash
>>$ sudo apt-get update
>>$ sudo apt-get install python-catkin-tools
```

If we are currently using the *catkin_make* build system we should delete the *devel* and *build* directories in our *catkin_ws* before running *catkin build*. Next, we can type the following inside of our *catkin_ws* to compile and run all tests from all packages:

```bash
>>$ catkin run_tests
```

The following line can be modified to specify individual packages as such:

```bash
>>$ catkin run_tests <package_name>
```

Finally, if we navigate to any ROS package within our work space we can run all of the tests associated with the package with the following command:

```bash
>>$ catkin run_tests --this
```

Results can be visualized by typing in the following command:

```bash
>>$ catkin_test_results
```

This will show descriptive messages based on how many tests were run, errors, failures, skipped tests, and the respective package where the failure occurred. However, when running tests with *catkin_tools* some plugins will lose functionality such as Pytest Clarity.


# License

For license information, please see the LICENSE files.
