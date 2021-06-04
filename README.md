![](./images/banner.png)

## Experimental ROS Noetic, Python 3, Ubuntu 20.04 Development Branch

This is an experimental development branch that we are using to port *stretch_ros* to ROS Noetic, Python 3, and Ubuntu 20.04. We plan to begin shipping this version preinstalled on Stretch RE1 robots at the end of the summer of 2021. It is close to being in a usable state, but **we don't recommend trying it yet**. This is because, first, there has been limited testing of this port, so standard use may result in unexected behaviors. And second, the installation **requires Ubuntu 20.04 on a second partition** on your robot. As such, it's a significant installation process.

In the near future, we expect to begin using the port internally at Hello Robot to continue refining it while adding new capabilities. We also expect some customers who have requested this port to begin trying it out. For users who wish to test out the Noetic port, please see the [installation guide](install_noetic.md). **This guide is in active development as well. Please proceed with caution.**

If you do begin using this branch, **please file issues here and ask general questions on the [forum](forum.hello-robot.com)**.

## Known Issues

 - There is no support for the Respeaker Microphone Array in Noetic yet.
 - There is no support for the Dexterous Wrist yet.


---

## Directories

The *stretch_ros* repository holds ROS related code for the Stretch RE1 mobile manipulator from Hello Robot Inc. For an overview of the capabilities in this repository, we recommend you look at the following forum post: https://forum.hello-robot.com/t/autonomy-video-details


| Resource                                                     | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
[hello_helpers](https://github.com/hello-robot/stretch_ros/blob/master/hello_helpers/README.md) | Miscellaneous helper code used across the stretch_ros repository 
[stretch_calibration](https://github.com/hello-robot/stretch_ros/tree/master/stretch_calibration/README.md) | Creates and updates calibrated URDFs for the Stretch RE1      
[stretch_core](https://github.com/hello-robot/stretch_ros/tree/master/stretch_core/README.md) | Enables basic use of the Stretch RE1 from ROS    
[stretch_deep_perception](https://github.com/hello-robot/stretch_ros/blob/master/stretch_deep_perception/README.md) | Demonstrations that use open deep learning models to perceive the world 
[stretch_demos](https://github.com/hello-robot/stretch_ros/tree/master/stretch_demos/README.md) | Demonstrations of simple autonomous manipulation  
[stretch_description](https://github.com/hello-robot/stretch_ros/blob/master/stretch_description/README.md) | Generate and export URDFs 
[stretch_funmap](https://github.com/hello-robot/stretch_ros/blob/master/stretch_funmap/README.md) | Demonstrations of Fast Unified Navigation, Manipulation And Planning (FUNMAP) 
[stretch_navigation](https://github.com/hello-robot/stretch_ros/blob/master/stretch_navigation/README.md) | Support for the ROS navigation stack, including move_base, gmapping, and AMCL.

## Licenses

This software is intended for use with the Stretch RE1 mobile manipulator, which is a robot produced and sold by Hello Robot Inc. For further information, including inquiries about dual licensing, please contact Hello Robot Inc.

For license details for this repository, see the LICENSE files found in the directories. A summary of the licenses follows: 

Directory | License
--- | ---
hello_helpers | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_calibration | [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html)
stretch_core | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_deep_perception | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_demos | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_description | [CC BY-NC-SA 4.0](https://creativecommons.org/licenses/by-nc-sa/4.0/)
stretch_funmap | [LGPLv3](https://www.gnu.org/licenses/lgpl-3.0.en.html)
stretch_navigation | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)

