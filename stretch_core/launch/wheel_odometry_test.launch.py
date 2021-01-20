from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare

# TODO(JafarAbdi): Check if the following args are needed, they're not being used
configurable_parameters = [
    {'name': 'urdf_file', 'default': PathJoinSubstitution([FindPackageShare('stretch_description'),
                                                           'urdf',
                                                           'stretch.urdf'])},
    {'name': 'controller_yaml_file', 'default': PathJoinSubstitution([FindPackageShare('stretch_core'),
                                                                      'config',
                                                                      'controller_calibration_head.yaml'])},
    {'name': 'calibration_directory', 'default': PathJoinSubstitution([EnvironmentVariable('HELLO_FLEET_PATH'),
                                                                       EnvironmentVariable('HELLO_FLEET_ID'),
                                                                       'calibration_ros'])}]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default']) for param in parameters]


def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():
    # REALSENSE D435i
    d435i_low_resolution_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ThisLaunchFileDir(), 'd435i_low_resolution.launch.py'])))

    d435i_configure_node = Node(package='stretch_core',
                                executable='d435i_configure',
                                output='screen',
                                parameters=[{'initial_mode': 'High Accuracy'}]
                                )

    # STRETCH DRIVER
    stretch_driver_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ThisLaunchFileDir(), 'stretch_driver.launch.py'])),
        launch_arguments={'broadcast_odom_tf': 'true',
                          'fail_out_of_range_goal': 'false'}.items())

    # TODO(JafarAbdi): This's not ported yet to ROS2
    # LASER RANGE FINDER
    # rplidar_launch_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(PathJoinSubstitution([ThisLaunchFileDir(), 'rplidar.launch.py'])))

    # KEYBOARD TELEOP
    keyboard_teleop_node = Node(package='stretch_core',
                                executable='keyboard_teleop',
                                output='screen'
                                )

    # VISUALIZE
    rviz_config_file = PathJoinSubstitution([FindPackageShare('stretch_core'),
                                             'rviz',
                                             'wheel_odometry_test.rviz'])
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='screen',
                     arguments=['-d', rviz_config_file]
                     )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) +
                             [d435i_low_resolution_launch_description,
                              d435i_configure_node,
                              stretch_driver_launch_description,
                              keyboard_teleop_node,
                              rviz_node])
