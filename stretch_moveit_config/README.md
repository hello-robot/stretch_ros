![](../images/banner.png)

## Stretch & MoveIt!

MoveIt is the standard ROS manipulation platform, and this package is the configuration for working with the Gazebo simulated Stretch with the MoveIt framework.

**Warning:** This package does not support running MoveIt! plans/trajectories on the actual Stretch robot. This package powers a demo of running MoveIt! plans/trajectories on a Gazebo simulated Stretch. [Details here](#hardware-integration).

### Offline Demo

To experiment with the planning capabilities of MoveIt on Stretch, you can run a demo _without_ Stretch hardware.

    roslaunch stretch_moveit_config demo.launch

This will allow you to move the robot around using interactive markers and create plans between poses.

### Hardware Integration

There is no planned support to run MoveIt on Stretch hardware. Instead, support for running MoveIt 2 (the successor to MoveIt) on Stretch hardware is available in [Stretch's ROS2 packages](https://github.com/hello-robot/stretch_ros2/).

## License

For license information, please see the LICENSE files.
