![](../images/banner.png)

## Overview

*stretch_navigation* provides the standard ROS navigation stack as two launch files. This package utilizes gmapping, move_base, and AMCL to drive the stretch RE1 around a mapped space. Running this code will require the robot to be untethered.

## Setup

These set up instructions are already performed on Stretch RE1 robots. Follow these instructions if *stretch_navigation* is not present in your ROS workspace or you are simulating Stretch on external hardware. Clone stretch_ros to your catkin workspace. Then install dependencies and build the packages, with the following set of commands:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/hello-robot/stretch_ros
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Quickstart

The first step is to map the space that the robot will navigate in. The `mapping.launch` will enable you to do this. First run:

```bash
roslaunch stretch_navigation mapping.launch
```

Rviz will show the robot and the map that is being constructed. With the terminal open, use the instructions printed by the teleop package to teleoperate the robot around the room. Avoid sharp turns and revisit previously visited spots to form loop closures. In Rviz, once you see a map that has reconstructed the space well enough, you can run the following commands to save the map to `stretch_user/`.

```bash
mkdir -p ~/stretch_user/maps
rosrun map_server map_saver -f ${HELLO_FLEET_PATH}/maps/<map_name>
```

The `<map_name>` does not include an extension. Map_saver will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

Next, with `<map_name>.yaml`, we can navigate the robot around the mapped space. Run:

```bash
roslaunch stretch_navigation navigation.launch map_yaml:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

Rviz will show the robot in the previously mapped space, however, it's likely that the robot's location in the map does not match the robot's location in the real space. In the top bar of Rviz, use 2D Pose Estimate to lay an arrow down roughly where the robot is located in the real space. AMCL, the localization package, will better localize our pose once we give the robot a 2D Nav Goal. In the top bar of Rviz, use 2D Nav Goal to lay down an arrow where you'd like the robot to go. In the terminal, you'll see move_base go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior: spinning around 360 degrees in place.

It is also possible to send 2D Pose Estimates and Nav Goals programatically. In your own launch file, you may include `navigation.launch` to bring up the navigation stack. Then, you can send `move_base_msgs::MoveBaseGoal` messages in order to navigate the robot programatically.

### Running in Simulation

To perform mapping and navigation in the Gazebo simulation of Stretch, substitute the `mapping_gazebo.launch` and `navigation_gazebo.launch` launch files into the commands above. The default Gazebo environment is the Willow Garage HQ. Use the "world" ROS argument to specify the Gazebo world within which to spawn Stretch.

```bash
roslaunch stretch_navigation mapping_gazebo.launch gazebo_world:=worlds/willowgarage.world
```

### Teleop using a Joystick Controller

The mapping launch files, `mapping.launch` and `mapping_gazebo.launch` expose the ROS argument, "teleop_type". By default, this ROS arg is set to "keyboard", which launches keyboard teleop in the terminal. If the xbox controller that ships with Stretch RE1 is plugged into your computer, the following command will launch mapping with joystick teleop:

```bash
roslaunch stretch_navigation mapping.launch teleop_type:=joystick
```

### Using ROS Remote Master

If you have set up [ROS Remote Master](https://docs.hello-robot.com/untethered_operation/#ros-remote-master) for [untethered operation](https://docs.hello-robot.com/untethered_operation/), you can use Rviz and teleop locally with the following commands:

```bash
# On Robot
roslaunch stretch_navigation mapping.launch rviz:=false teleop_type:=none

# On your machine, Terminal 1:
rviz -d `rospack find stretch_navigation`/rviz/mapping.launch
# On your machine, Terminal 2:
roslaunch stretch_core teleop_twist.launch teleop_type:=keyboard # or use teleop_type:=joystick if you have a controller
```

## License

For license information, please see the LICENSE files.
