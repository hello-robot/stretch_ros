![](../images/banner.png)

## Stretch & MoveIt!
MoveIt is the standard ROS manipulation platform, and this package is the configuration for working with Stretch with the MoveIt framework.


### Offline Demo

To experiment with the planning capabilities of MoveIt on Stretch, you can run a demo _without_ Stretch hardware.

    roslaunch stretch_moveit_config demo.launch

This will allow you to move the robot around using interactive markers and create plans between poses.

### Hardware Integration

To run MoveIt with the actual hardware, (assuming `stretch_driver` is already running) simply run

    roslaunch stretch_moveit_config move_group.launch

This will runs all of the planning capabilities, but without the setup, simulation and interface that the above demo provides. In order to create plans for the robot with the same interface as the offline demo, you can run

    roslaunch stretch_moveit_config moveit_rviz.launch

## License

For license information, please see the LICENSE files.
