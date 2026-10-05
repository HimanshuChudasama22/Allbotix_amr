from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('amr_mapping'),
        'config',
        'ultrasound.yaml',
    )

    # Keep these mounts identical to navigation.launch.py.
    us_center_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_us_center',
        arguments=[
            '--x', '0.25',
            '--y', '0.0',
            '--z', '0.02519',
            '--roll', '0.0',
            '--pitch', '-0.15',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'us_center',
        ],
    )

    us_right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_us_right',
        arguments=[
            '--x', '0.225',
            '--y', '-0.225',
            '--z', '0.02519',
            '--roll', '0.0',
            '--pitch', '-0.15',
            '--yaw', '-0.7854',
            '--frame-id', 'base_link',
            '--child-frame-id', 'us_right',
        ],
    )

    us_left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_us_left',
        arguments=[
            '--x', '0.225',
            '--y', '0.225',
            '--z', '0.02519',
            '--roll', '0.0',
            '--pitch', '-0.15',
            '--yaw', '0.7854',
            '--frame-id', 'base_link',
            '--child-frame-id', 'us_left',
        ],
    )

    ultrasound_center = Node(
        package='amr_mapping',
        executable='a21_ultrasound_node',
        name='a21_ultrasound_center',
        output='screen',
        parameters=[config],
    )

    ultrasound_right = Node(
        package='amr_mapping',
        executable='a21_ultrasound_node',
        name='a21_ultrasound_right',
        output='screen',
        parameters=[config],
    )

    ultrasound_left = Node(
        package='amr_mapping',
        executable='a21_ultrasound_node',
        name='a21_ultrasound_left',
        output='screen',
        parameters=[config],
    )

    range_filter = Node(
        package='amr_mapping',
        executable='range_filter',
        name='range_filter',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('amr_mapping'),
            'config',
            'range_filter.yaml',
        )],
    )

    range_to_scan = Node(
        package='amr_mapping',
        executable='range_to_scan',
        name='range_to_scan',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('amr_mapping'),
            'config',
            'range_to_scan.yaml',
        )],
    )

    return LaunchDescription([
        us_center_tf,
        us_right_tf,
        us_left_tf,
        ultrasound_center,
        ultrasound_right,
        ultrasound_left,
        range_filter,
        range_to_scan,
    ])
