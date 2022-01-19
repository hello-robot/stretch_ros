![](../images/banner.png)

## Stretch & MoveIt!
MoveIt is the standard ROS manipulation platform, and this package is the configuration for working with Stretch with the MoveIt framework.


### Offline Demo

To experiment with the planning capabilities of MoveIt on Stretch, you can run a demo _without_ Stretch hardware.

    roslaunch stretch_moveit_config demo.launch

This will allow you to move the robot around using interactive markers and create plans between poses.

### Hardware Integration

There is no planned support to run MoveIt on Stretch hardware. Instead, support for running MoveIt 2 (the successor to MoveIt) on Stretch hardware is being developed in [Stretch's ROS2 packages](https://github.com/hello-robot/stretch_ros2/). The primary reason to support MoveIt 2 instead of MoveIt 1 is because MoveIt 2 introduces planning for differential drive bases, whereas MoveIt 1 does not have this ability. Manipulation with Stretch is more capable when the mobile base is included.

Please keep an eye on [Stretch's ROS2 packages](https://github.com/hello-robot/stretch_ros2/) and our [forum](https://forum.hello-robot.com/) to track the state of Stretch + MoveIt 2 support.

## License

For license information, please see the LICENSE files.
