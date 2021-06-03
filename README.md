![](./images/banner.png)

## Experimental ROS Noetic, Python 3, Ubuntu 20.04 Development Branch

This is an experimental development branch that we are using to port *stretch_ros* to ROS Noetic, Python 3, and Ubuntu 20.04. We plan to begin shipping this version preinstalled on Stretch RE1 robots at the end of the summer of 2021. It is close to being in a usable state. but **we don't recommend trying it yet**. 

In the near future, we expect to begin using it internally at Hello Robot to continue refining it while adding new capabilities. We also expect some customers who have requested this port to begin trying it out. However, we have more work to do before this happens, including writing installation documentation.

If you decide to use this branch, **you will need to install Ubuntu 20.04 on a second partition on your robot**. As such, it's a significant installation process. 

If you do begin using this branch, **please file issues here and ask general questions on the forum**. 

## Installation Instructions

1. Install a partition with 20.04
    1. Download and write the Ubuntu 20.04 iso file to a USB key.
    2. Make sure you have sufficient space on your robot’s solid state drive (SSD) and backup all of your critical files.
    3. Boot the robot with the USB key plugged into a USB port in the robot’s trunk. 
    4. The installer should detect your Ubuntu 18.04 installation and provide the option to install Ubuntu 20.04 alongside Ubuntu 18.04. This option will be selected by default. 
    5. Proceeding with this option will result in a new partition with Ubuntu 20.04, although there must be sufficient available space on the robot’s SSD.
2. Copy materials from your Ubuntu 18.04 partition
   1. Boot into your Ubuntu 20.04 partition.
   2. Mount your Ubuntu 18.04 partition. For example, you can do this by going to “+ Other Locations” using the file explorer GUI and clicking on the Volume  associaed with your Ubuntu 18.04 partition. 


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

