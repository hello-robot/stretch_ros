import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import xacro


def generate_launch_description():
    robot_description_path = os.path.join(get_package_share_directory('stretch_description'),
                                          'urdf',
                                          'stretch.urdf')

    calibrated_controller_yaml_file = os.path.join(get_package_share_directory('stretch_core'),
                                                   'config',
                                                   'controller_calibration_head.yaml')

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 arguments=[robot_description_path],
                                 output='log',
                                 parameters=[{'source_list': ["/stretch/joint_states"]},
                                             {'rate': 15}])

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': xacro.process_file(robot_description_path).toxml()},
                                             {'publish_frequency': 15}])

    stretch_driver = Node(package='stretch_core',
                          executable='stretch_driver',
                          name='stretch_driver',
                          remappings=[('cmd_vel', '/stretch/cmd_vel'),
                                      ('joint_states', '/stretch/joint_states')],
                          parameters=[{'rate': 25.0},
                                      {'timeout': 0.5},
                                      {'controller_calibration_file': calibrated_controller_yaml_file},
                                      {'broadcast_odom_tf': LaunchConfiguration('broadcast_odom_tf')},
                                      {'fail_out_of_range_goal': LaunchConfiguration('fail_out_of_range_goal')}])

    return LaunchDescription([DeclareLaunchArgument('broadcast_odom_tf'),
                              DeclareLaunchArgument('fail_out_of_range_goal'),
                              joint_state_publisher,
                              robot_state_publisher,
                              stretch_driver])
