from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution


# HIGHEST RESOLUTION, but also has the highest minimum depth
# (280mm Min-Z) below which objects generate bad noise, such as
# when the arm and gripper are raised close to the camera.
def generate_launch_description():
    return LaunchDescription(
        # REALSENSE D435i
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([ThisLaunchFileDir(), 'd435i_basic.launch.py'])),
            launch_arguments={'depth_width': '1280',
                              'depth_height': '720',
                              'color_width': '1280',
                              'color_height': '720'}.items())])
