from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_amr_mapping = get_package_share_directory('amr_mapping')
    cartographer_config_dir = os.path.join(pkg_amr_mapping, 'config')
    cartographer_config_basename = 'cartographer.lua'

    default_map_filestem = os.path.join(
        os.path.expanduser('~'),
        'maps',
        'cartographer_map',
    )

    map_filestem_arg = DeclareLaunchArgument(
        'map_filestem',
        default_value=default_map_filestem,
        description=(
            'Path stem for autosaved map (writes .pgm + .yaml). '
            'Manual save: ros2 run nav2_map_server map_saver_cli -f <stem>'
        ),
    )

    # YDLIDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'ydlidar_ros2_driver'
                ),
                'launch',
                'ydlidar_launch.py'
            )
        )
    )

    # Motor controller + ros2_control
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'zlac8015d_hardware'
                ),
                'launch',
                'robot_control.launch.py'
            )
        )
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'orbbec_camera'
                ),
                'launch',
                'gemini_330_series.launch.py'
            )
        )
    )

    ultrasound_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_amr_mapping,
                'launch',
                'ultrasound.launch.py'
            )
        )
    )

    # Static transform:
    # base_link --> laser_frame
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=[
            '0.140', '0', '0.15',      # x y z
            '3.1416', '0', '0',         # roll pitch yaw
            'base_link',
            'laser_frame'
        ]
    )

    depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_depth',
        arguments=[
            '0.2237', '0', '0.08486',      # x y z
            '0', '-0.7854', '0',         # roll pitch yaw
            'base_link',
            'camera_link'
        ]
    )

    # Cartographer SLAM
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', cartographer_config_basename,
        ],
        remappings=[
            ('scan', '/scan_reliable'),
            ('odom', '/diff_drive_controller/odom'),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        # This executable reads gflags, not ROS parameters, for these options.
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    teleop_node = Node(
        package='amr_mapping',
        executable='teleop_node',
        output='screen'
    )

    cmd_vel_bridge = Node(
        package='amr_mapping',
        executable='cmd_vel_bridge',
        output='screen'
    )

    scan_relay_node = Node(
        package='amr_mapping',
        executable='scan_relay_node',
        output='screen'
    )

    # Caches /map and writes pgm+yaml on Ctrl+C / launch shutdown
    map_autosave_node = Node(
        package='amr_mapping',
        executable='map_autosave',
        name='map_autosave',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'map_filestem': LaunchConfiguration('map_filestem'),
        }],
    )

    return LaunchDescription([
        map_filestem_arg,
        LogInfo(msg=[
            'Manual map save: ros2 run nav2_map_server map_saver_cli -f ',
            LaunchConfiguration('map_filestem'),
        ]),
        joy_node,
        teleop_node,
        lidar_launch,
        motor_launch,
        # camera_launch,
        # ultrasound_launch,
        # depth_tf,
        laser_tf,
        scan_relay_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        map_autosave_node,
        # joy_node,
        # teleop_node,
        # cmd_vel_bridge,
    ])
