from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        # STRETCH DRIVER
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([ThisLaunchFileDir(), 'stretch_driver.launch.py'])),
            launch_arguments={'broadcast_odom_tf': 'true',
                              'fail_out_of_range_goal': 'false'}.items()),
        # KEYBOARD TELEOP
        Node(package='stretch_core',
             executable='keyboard_teleop',
             prefix='xterm -e',
             output='screen')])
