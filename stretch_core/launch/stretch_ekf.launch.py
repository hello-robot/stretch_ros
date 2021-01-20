from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([Node(package='robot_localization',
                                   executable='ekf_node',
                                   parameters=[PathJoinSubstitution([FindPackageShare('stretch_core'),
                                                                     'launch',
                                                                     'stretch_ekf.yaml'])],
                                   remappings=[
                                       # Placeholder for output topic remapping
                                       # ('odometry/filtered', ''),
                                       # ('accel/filtered', '')
                                   ])
                              ]
                             )
