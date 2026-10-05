from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    slam_config = os.path.join(
        get_package_share_directory('amr_mapping'),
        'config',
        'slam_toolbox.yaml'
    )
    pkg_slam_toolbox = get_package_share_directory(
            'slam_toolbox'
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
                get_package_share_directory('amr_mapping'),
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
            name='static_tf_pub_laser',
            arguments=[
                '0.2237', '0', '0.08486',      # x y z
                '0', '-0.7854', '0',         # roll pitch yaw
                'base_link',
                'camera_link'
            ]
        )

    # SLAM
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config]
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

    wheel_control_node = Node(
    
        package='amr_mapping',
        executable='wheel_control_node',

        output='screen'
    )

    scan_relay = Node(
        
        package='amr_mapping',
        executable='scan_relay',

        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(
                pkg_slam_toolbox,
                'rviz',
                'slam_toolbox_default.rviz'
            )
        ],
    )

    return LaunchDescription([
        lidar_launch,
        motor_launch,
        camera_launch,
        ultrasound_launch,
        depth_tf,
        # wheel_control_node,
        laser_tf,
        scan_relay,
        slam_node,
        joy_node,
        teleop_node,
        cmd_vel_bridge,
        # rviz_node,
    ])