![](../images/banner.png)

# Overview

*stretch_core* provides the core ROS interfaces to the Stretch RE1 mobile manipulator from Hello Robot Inc. It includes the following nodes:

*stretch_driver* : node that communicates with the low-level Python library (stretch_body) to interface with the Stretch RE1

*detect_aruco_markers* : node that detects and estimates the pose of ArUco markers, including the markers on the robot's body

*d435i_** : various nodes to help use the Stretch RE1's 3D camera

*keyboard_teleop* : node that provides a keyboard interface to control the robot's joints

# API

## Nodes

### [stretch_driver](./nodes/stretch_driver)

#### Parameters

##### broadcast_odom_tf

If set to true, stretch_driver will publish an odom to base_link TF.

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
