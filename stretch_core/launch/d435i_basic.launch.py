from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

# https://github.com/intel-ros/realsense
launch_arguments = {'accel_fps': '63',
                    'gyro_fps': '200',
                    'depth_fps': '15',
                    'enable_infra1': 'false',
                    'enable_infra2': 'false',
                    'enable_accel': 'true',
                    'depth_width': LaunchConfiguration('depth_width'),
                    'depth_height': LaunchConfiguration('depth_height'),
                    'color_width': LaunchConfiguration('color_width'),
                    'color_height': LaunchConfiguration('color_height'),
                    'color_fps': '15',
                    # publish depth streams aligned to other streams
                    'align_depth': 'true',
                    # publish an RGBD point cloud
                    'filters': 'pointcloud',
                    # enable_sync: gathers closest frames of different sensors,
                    #              infra red, color and depth, to be sent with the same
                    #              timetag. This happens automatically when such filters as
                    #              pointcloud are enabled.
                    'enable_sync': 'true',
                    # You can have a full depth to pointcloud, coloring the regions beyond the texture with zeros...
                    # Set to true in order to make use of the full field of view of
                    # the depth image instead of being restricted to the field of
                    # view associated with the narrower RGB camera. Note that
                    # points out of the RGB camera's field of view will have their
                    # colors set to 0,0,0.
                    'allow_no_texture_points': 'true'}.items()


def generate_launch_description():
    d435i_accel_correction_node = Node(package='stretch_core',
                                       executable='d435i_accel_correction',
                                       output='screen')

    # "The D435i depth camera generates and transmits the gyro and
    # accelerometer samples independently, as the inertial sensors
    # exhibit different FPS rates (200/400Hz for gyro, 63/250Hz for
    # accelerometer)."
    # https://realsense.intel.com/how-to-getting-imu-data-from-d435i-and-t265/
    realsense2_camera_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'config', 'rs_launch.py'])),
        launch_arguments=launch_arguments)

    return LaunchDescription([DeclareLaunchArgument('depth_width'),
                              DeclareLaunchArgument('depth_height'),
                              DeclareLaunchArgument('color_width'),
                              DeclareLaunchArgument('color_height'),
                              d435i_accel_correction_node,
                              realsense2_camera_node])
