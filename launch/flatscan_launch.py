from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bea_sensors',
            namespace='bea_sensors',
            executable='flatscan',
            name='flatscan',
            output='screen',
            parameters=[
                {"port": "/dev/ttyUSB0"},
                {"baudrate": "921600"},
                {"scan_frame_id": "laser_link"},
                {"scan_topic": "/scan"},
                {"heartbeat_topic": "/heartbeat"},
                {"emergency_topic": "/emergency"},
                {"min_range": 0.},  # in meters
                {"max_range": 8.},  # in meters
                {"enable_temperature": 1},
                {"information_in_mdi": 0},
                {"detection_field_mode": "HD"},
                {"optimization": 0},
                {"angle_first": 0.},  # in degrees
                {"angle_last": 108.},  # in degrees
                {"enable_counter": 1},
                {"heartbeat_period": 5},  # in seconds
                {"enable_facet": 1},
                {"averaging_setting": 0}
            ]
        ),
        Node(package='rviz2', namespace='', executable='rviz2', name='rviz2',
             arguments=['-d', [os.path.join(get_package_share_directory('bea_sensors'), 'launch', 'flatscan.rviz')]])
    ])
