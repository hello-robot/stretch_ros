![](../images/HelloRobotLogoBar.png)

## Overview

*stretch_core* provides the core ROS interfaces to the Stretch RE1 mobile manipulator from Hello Robot Inc. It includes the following nodes: 

*stretch_driver* : node that communicates with the low-level Python library (stretch_body) to interface with the Stretch RE1

*detect_aruco_markers* : node that detects and estimates the pose of ArUco markers, including the markers on the robot's body

*d435i_** : various nodes to help use the Stretch RE1's 3D camera

*keyboard_teleop* : node that provides a keyboard interface to control the robot's joints

## Testing

The primary testing framework being used within *stretch_ros* is pytest. Running tests are depedent on the catkin_tools library. Thus we can assume we will be using *catkin build* instead of *catkin_make*. To install catkin_tools first ensure that you have the ROS repositories that contain .deb and catkin_tools: 

```bash
>>$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
>>$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

Next you can install catkin_tools by running:

```bash
>>$ sudo apt-get update
>>$ sudo apt-get install python-catkin-tools
```

If you are currently using the *catkin_make* build system you should delete the devel and build directories in your catkin_ws before running *catkin build*. Once catkin_tools is up and running you should then ensure that pytest is installed by running:

```bash
>>$ pip3 install -U pytest
```

And you can ensure the version is correct by running: 

```bash
>>$ pytest --version
pytest 6.2.4
```

Testing can be split into four different categories: 

1. Python library unit tests (pytest)
2. ROS node unit tests (pytest + rostest)
3. ROS node integration tests (pytest + rostest)
4. ROS functional tests (pytest + rostest)

Tests suites are organized using the rostest framework which is an extention on standard ROS launch files. The runner script converts the output file parameter to the pytest supported format, additionally, it passes the test directory to pytest. Test suites can be created simply by adding the following line to the desired test file: 

Python Library Tests: 

```html
<launch>
  <param name="test_module" value="../src"/>
  <test test-name="test_lib" pkg="my_pkg" type="pytest_runner.py" />
</launch>
```

ROS Node Unit Tests: 

```html 
<launch>
  <node pkg="my_pkg" type="publisher" name="publisher" />
  <param name="test_module" value="listener"/>
  <test test-name="test_listener" pkg="my_pkg" type="pytest_runner.py" />
</launch>
```

Next, you can type the following inside of your catkin_ws to compile and run all tests from all packages:

```bash
>>$ catkin run_tests
```

The following line can be modifed to specify indvidual packages as such: 

```bash
>>$ catkin run_tests <package_name>
```

Finally, if you navigate to any ROS package within your work space you can run all of the tests associated with the package with the following command: 

```bash 
>>$ catkin run_tests --this 
```

Results can be visualized by typing in the following command: 

```bash
>>$ catkin_test_results
```

This will show descriptive messages based on how many tests were run, errors, failures, skipped tests, and the respective pacakge where the failure occured. 


## License

For license information, please see the LICENSE files. 
