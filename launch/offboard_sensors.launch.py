#!/usr/bin/env python3
"""
AMR Offboard Sensors Launch
Starts only robot-side data publishers for remote processing on another PC.
No SLAM Toolbox and no Nav2.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_slam')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/arduino',
        description='Arduino serial port'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ydlidar',
        description='YDLidar serial port'
    )
    publish_reliable_scan_arg = DeclareLaunchArgument(
        'publish_reliable_scan', default_value='true',
        description='Run scan relay to publish /scan_reliable from /scan'
    )

    arduino_bridge_node = Node(
        package='amr_slam',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'ticks_per_rev': 2350,
            'wheel_diameter_m': 0.05895,
            'wheelbase_m': 0.05895*(0.375/0.0669),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
        }],
    )

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(pkg_share, 'config', 'ydlidar_params.yaml'),
            {'port': LaunchConfiguration('lidar_port')},
        ],
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.085139',
            '--y', '0.0',
            '--z', '0.0',
            '--yaw', '3.14159262358',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame',
        ],
    )

    scan_relay_node = Node(
        package='amr_slam',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
        parameters=[{
            'enable_sector_mask': False,
            'blocked_sectors_deg': [-120.0, -60.0, 60.0, 120.0],
        }],
        condition=IfCondition(LaunchConfiguration('publish_reliable_scan')),
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        publish_reliable_scan_arg,
        arduino_bridge_node,
        ydlidar_node,
        base_to_laser_tf,
        scan_relay_node,
    ])
