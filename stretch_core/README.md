![](../images/banner.png)

## Overview

*stretch_core* provides the core ROS interfaces to the Stretch RE1 mobile manipulator from Hello Robot Inc. It includes the following nodes:

*stretch_driver* : node that communicates with the low-level Python library (stretch_body) to interface with the Stretch RE1

*detect_aruco_markers* : node that detects and estimates the pose of ArUco markers, including the markers on the robot's body

*d435i_** : various nodes to help use the Stretch RE1's 3D camera

*keyboard_teleop* : node that provides a keyboard interface to control the robot's joints

## Testing

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


## License

For license information, please see the LICENSE files.
