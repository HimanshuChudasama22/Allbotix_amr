#!/usr/bin/env python3
"""
AMR SLAM Mapping Launch File
Starts: Arduino bridge, YDLidar X2, static TFs, scan relay, SLAM Toolbox, and optionally RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, IncludeLaunchDescription
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
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2', default_value='true',
        description='Launch Nav2 navigation stack'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to Nav2 parameters file'
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
            'enable_sector_mask':False,
            # Side beams blocked by robot structure (degrees, LaserScan frame)
            'blocked_sectors_deg': [-120.0, -60.0, 60.0, 120.0],
        }],
    )

    # ── SLAM Toolbox (Online Async) ─────────────────────────────
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    # ── SLAM lifecycle bringup with retries (avoids startup race) ──
    slam_lifecycle_bringup = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            """
            configured=0
            for i in $(seq 1 30); do
              if ros2 lifecycle set /slam_toolbox configure; then
                configured=1
                break
              fi
              sleep 1
            done
            [ "$configured" -eq 1 ] || exit 1

            activated=0
            for i in $(seq 1 30); do
              if ros2 lifecycle set /slam_toolbox activate; then
                activated=1
                break
              fi
              sleep 1
            done
            [ "$activated" -eq 1 ] || exit 1
            """,
        ],
        output='screen',
    )

    # ── Nav2 bringup (for goal navigation while mapping) ─────────
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_nav2')),
    )

    # Delay Nav2 start slightly so TF and SLAM lifecycle are up
    nav2_delayed_start = TimerAction(
        period=5.0,
        actions=[nav2_bringup_launch],
    )

    # ── Camera Publisher ────────────────────────────────────────
    camera_node = Node(
        package='amr_slam',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'device': '/dev/amr_camera',
            'frame_id': 'camera_frame',
            'width': 640,
            'height': 480,
            'fps': 15.0,
            'publish_compressed': True,
        }],
    )

    # Static TF: base_link -> camera_frame
    # Adjust x, y, z, yaw to your camera mounting position
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x', '0.1',
            '--y', '0.0',
            '--z', '0.05',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_frame',
        ],
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
        use_rviz_arg,
        use_nav2_arg,
        nav2_params_arg,
        arduino_bridge_node,
        ydlidar_node,
        base_to_laser_tf,
        scan_relay_node,
        slam_toolbox_node,
        slam_lifecycle_bringup,
        nav2_delayed_start,
        #camera_node,
        #base_to_camera_tf,
        rviz_node,
    ])
