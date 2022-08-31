![](./images/banner.png)

# Overview

The *stretch_ros* repository holds ROS related code for the Stretch RE1 mobile manipulator from Hello Robot Inc. 
For an overview of the capabilities in this repository, we recommend you look at the [following forum post](https://forum.hello-robot.com/t/autonomy-video-details).

**Please be aware that the code in this repository is currently under heavy development.** 


| Resource                                                     | Description                                                  |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
[hello_helpers](hello_helpers/README.md) | Miscellaneous helper code used across the stretch_ros repository
[stretch_calibration](stretch_calibration/README.md) | Creates and updates calibrated URDFs for the Stretch RE1
[stretch_core](stretch_core/README.md) | Enables basic use of the Stretch RE1 from ROS
[stretch_deep_perception](stretch_deep_perception/README.md) | Demonstrations that use open deep learning models to perceive the world
[stretch_demos](stretch_demos/README.md) | Demonstrations of simple autonomous manipulation
[stretch_description](stretch_description/README.md) | Generate and export URDFs
[stretch_funmap](stretch_funmap/README.md) | Demonstrations of Fast Unified Navigation, Manipulation And Planning (FUNMAP)
[stretch_gazebo](stretch_gazebo/README.md) | Support for simulation of Stretch in the Gazebo simulator
[stretch_moveit_config](stretch_gazebo/README.md) | Config files to use Stretch with the MoveIt Motion Planning Framework
[stretch_navigation](stretch_navigation/README.md) | Support for the ROS navigation stack, including move_base, gmapping, and AMCL



## Code Status & Development Plans

We intend for the following high-level summary to provide guidance about the current state of the code and planned development activities.

Directory | Testing Status | Notes 
--- | --- | ---
hello_helpers | GOOD |
stretch_calibration | GOOD |
stretch_core | GOOD | 
stretch_deep_perception | GOOD |
stretch_demos | FAIR | 
stretch_description | GOOD |
stretch_funmap | FAIR | 
stretch_gazebo | FAIR | differs from stretch_core in its underlying controllers
stretch_moveit_config | FAIR | does not support mobile base planning in ROS 1
stretch_navigation | GOOD |

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
stretch_description | [BSD 3-Clause Clear License](https://choosealicense.com/licenses/bsd-3-clause-clear/)
stretch_funmap | [LGPLv3](https://www.gnu.org/licenses/lgpl-3.0.en.html)
stretch_gazebo | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_moveit_config | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)
stretch_navigation | [Apache 2.0](http://www.apache.org/licenses/LICENSE-2.0)

