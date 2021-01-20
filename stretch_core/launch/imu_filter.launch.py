from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter', output='screen',
            remappings=[('/imu/data_raw', '/imu_mobile_base'),
                        ('/imu/mag', '/magnetometer_mobile_base')],
            parameters=[{'use_mag': False,
                         'fixed_frame': 'map'}]
        )
    ])