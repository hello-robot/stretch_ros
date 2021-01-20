from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARUCO MARKER DETECTOR
    return LaunchDescription([Node(package='stretch_core',
                                   executable='detect_aruco_markers',
                                   output='screen',
                                   parameters=[PathJoinSubstitution([FindPackageShare('stretch_core'),
                                                                     'config',
                                                                     'stretch_marker_dict.yaml'])]
                                   )
                              ])
