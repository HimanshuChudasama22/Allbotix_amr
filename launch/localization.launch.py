#!/usr/bin/env python3
"""
AMR Localization Launch File
Starts: Arduino bridge, YDLidar X2, static TFs, scan relay, Nav2 localization + navigation, and optionally RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_slam')

    # ── Launch Arguments ────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/arduino',
        description='Arduino serial port'
    )
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ydlidar',
        description='YDLidar serial port'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'map', 'office_map.yaml'),
        description='Full path to the map yaml file'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )
    auto_global_localization_arg = DeclareLaunchArgument(
        'auto_global_localization', default_value='true',
        description='Call /reinitialize_global_localization automatically after startup'
    )

    # ── Arduino Bridge Node ─────────────────────────────────────
    arduino_bridge_node = Node(
        package='amr_slam',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'ticks_per_rev': 2350,
            'wheel_diameter_m': 0.0635,
            'wheelbase_m': 0.0635*(0.375/0.0665),
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True,
        }],
    )

    # ── YDLidar X2 Node ────────────────────────────────────────
    ydlidar_params_file = os.path.join(pkg_share, 'config', 'ydlidar_params.yaml')
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            ydlidar_params_file,
            {'port': LaunchConfiguration('lidar_port')},
        ],
    )

    # ── Static Transform: base_link -> laser_frame ──────────────
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

    # ── Scan QoS Relay (BestEffort -> Reliable) ───────────────
    scan_relay_node = Node(
        package='amr_slam',
        executable='scan_relay',
        name='scan_relay',
        output='screen',
        parameters=[{
            'enable_sector_mask': False,
            'blocked_sectors_deg': [-120.0, -60.0, 60.0, 120.0],
        }],
    )

    # ── Nav2 Bringup: Localization + Navigation on static map ──
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'slam': 'False',
            'use_localization': 'True',
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
    )

    nav2_delayed_start = TimerAction(
        period=5.0,
        actions=[nav2_bringup_launch],
    )

    # ── Auto AMCL global localization (no manual 2D pose estimate) ──
    auto_global_localization = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            """
            for i in $(seq 1 30); do
              if ros2 service type /reinitialize_global_localization >/dev/null 2>&1; then
                ros2 service call /reinitialize_global_localization std_srvs/srv/Empty "{}" && exit 0
              fi
              sleep 1
            done
            echo "Timed out waiting for /reinitialize_global_localization"
            exit 1
            """,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_global_localization')),
    )

    auto_global_localization_delayed = TimerAction(
        period=12.0,
        actions=[auto_global_localization],
    )

    # ── RViz2 (optional) ───────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'slam_view.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        serial_port_arg,
        lidar_port_arg,
        map_arg,
        nav2_params_arg,
        use_rviz_arg,
        auto_global_localization_arg,
        arduino_bridge_node,
        ydlidar_node,
        base_to_laser_tf,
        scan_relay_node,
        nav2_delayed_start,
        auto_global_localization_delayed,
        rviz_node,
    ])
