'''
Author: playboy_plus 793879445@qq.com
Date: 2025-11-12 11:00:28
LastEditors: playboy_plus 793879445@qq.com
LastEditTime: 2026-01-05 11:29:18
FilePath: \bea_sensors\launch\flatscan_422.launch.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
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
                'communication': 'serial',
                'port': '/dev/ttyUSB0',
                'baudrate': 921600,
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
