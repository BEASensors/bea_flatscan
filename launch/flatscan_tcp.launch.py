from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bea_sensors')
    rviz_config_path = os.path.join(pkg_share, 'launch', 'flatscan.rviz')
    return LaunchDescription([
        Node(
            package='bea_sensors',
            executable='flatscan',
            name='flatscan',
            output='screen',
            parameters=[{
                'communication': 'ethernet',
                'ip': '192.168.1.232',
                'port': 2111,
                'scan_frame_id': 'laser_link',
                'scan_topic': '/scan',
                'heartbeat_topic': '/heartbeat',
                'emergency_topic': '/emergency',
                'min_range': 0.0,
                'max_range': 8.0,
                'enable_temperature': 1,
                'information_in_mdi': 2,
                'detection_field_mode': 'HD',
                'optimization': 0,
                'angle_first': 0.0,
                'angle_last': 108.0,
                'enable_counter': 1,
                'heartbeat_period': 5,
                'enable_facet': 1,
                'averaging_setting': 0,
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ]) 
