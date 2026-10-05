from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # =========================================================
    # Package directories
    # =========================================================

    pkg_amr_mapping = get_package_share_directory(
        'amr_mapping'
    )

    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup'
    )

    pkg_ydlidar = get_package_share_directory(
        'ydlidar_ros2_driver'
    )

    pkg_zlac_hardware = get_package_share_directory(
        'zlac8015d_hardware'
    )

    # =========================================================
    # Launch arguments
    # =========================================================

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_amr_mapping,
            'maps',
            'map.yaml'
        ),
        description='Full path to the saved map YAML file'
    )

    params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(
            pkg_amr_mapping,
            'config',
            'nav2_params.yaml'
        ),
        description='Full path to the Nav2 parameters file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use real system time for the physical robot'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate Nav2 lifecycle nodes'
    )

    # =========================================================
    # Launch configurations
    # =========================================================

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('nav2_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # =========================================================
    # Physical YDLIDAR driver
    # =========================================================

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ydlidar,
                'launch',
                'ydlidar_launch.py'
            )
        )
    )

    # =========================================================
    # ZLAC motor driver and ros2_control
    # =========================================================

    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_zlac_hardware,
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

    depth_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=[
                '0.2537', '0', '0.08486',    #'0.2237', '0', '0.08486',      # x y z
                '0', '-0.8054', '0',         #'0', '-0.7854', '0',         # roll pitch yaw
                'base_link',
                'camera_link'
            ]
        )

    ultrasound_config = os.path.join(
        pkg_amr_mapping,
        'config',
        'ultrasound.yaml',
    )

    ultrasound_center = Node(
        package='amr_mapping',
        executable='a21_ultrasound_node',
        name='a21_ultrasound_center',
        output='screen',
        parameters=[ultrasound_config],
    )

    ultrasound_right = Node(
        package='amr_mapping',
        executable='a21_ultrasound_node',
        name='a21_ultrasound_right',
        output='screen',
        parameters=[ultrasound_config],
    )

    ultrasound_left = Node(
        package='amr_mapping',
        executable='a21_ultrasound_node',
        name='a21_ultrasound_left',
        output='screen',
        parameters=[ultrasound_config],
    )

    range_filter = Node(
        package='amr_mapping',
        executable='range_filter',
        name='range_filter',
        output='screen',
        parameters=[os.path.join(
            pkg_amr_mapping,
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
            pkg_amr_mapping,
            'config',
            'range_to_scan.yaml',
        )],
    )

    # =========================================================
    # Static transform: base_link -> laser_frame
    #
    # Position:
    # x = 0.195 m forward
    # y = 0.0 m
    # z = 0.15 m upward
    #
    # Change roll to 3.1416 only when the lidar is upside down.
    # =========================================================

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        output='screen',
        arguments=[
            '--x', '0.185',
            '--y', '0.0',
            '--z', '0.15',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '3.1416',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame',
        ]
    )

    # =========================================================
    # Static transforms: base_link -> ultrasound frames
    # Approximate front mounts (x forward, y left, z up).
    # =========================================================

    us_center_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_us_center',
        output='screen',
        arguments=[
            '--x', '0.26',
            '--y', '0.0',
            '--z', '0.02519',
            '--roll', '0.0',
            '--pitch', '-0.15', #'-0.15',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'us_center',
        ]
    )

    us_right_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_us_right',
        output='screen',
        arguments=[
            '--x', '0.235',
            '--y', '-0.225',
            '--z', '0.02519',
            '--roll', '0.0',
            '--pitch', '-0.15',
            '--yaw', '-0.7854',
            '--frame-id', 'base_link',
            '--child-frame-id', 'us_right',
        ]
    )

    us_left_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_us_left',
        output='screen',
        arguments=[
            '--x', '0.235',
            '--y', '0.225',
            '--z', '0.02519',
            '--roll', '0.0',
            '--pitch', '-0.15',
            '--yaw', '0.7854',
            '--frame-id', 'base_link',
            '--child-frame-id', 'us_left',
        ]
    )

    # =========================================================
    # Joystick driver
    # =========================================================

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    # =========================================================
    # Custom joystick teleoperation node
    # =========================================================

    teleop_node = Node(
        package='amr_mapping',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    # The slowdown velocity smoother publishes /cmd_vel_safe.
    cmd_vel_bridge = Node(
        package='amr_mapping',
        executable='cmd_vel_bridge',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_safe')],
    )
    scan_relay = Node(
    
        package='amr_mapping',
        executable='scan_relay_node',

        output='screen'
    )

    wheel_control_node = Node(
        
            package='amr_mapping',
            executable='wheel_control_node',
    
            output='screen'
        )

    # =========================================================
    # RViz, pre-loaded with the package config view.
    # =========================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(
                pkg_amr_mapping,
                'config',
                'nav2_at_base.rviz'
            )
        ],
    )

    # =========================================================
    # Complete Nav2 bringup
    #
    # Starts:
    # - map_server
    # - AMCL
    # - controller_server
    # - planner_server
    # - smoother_server
    # - behavior_server
    # - bt_navigator
    # - waypoint_follower
    # - velocity_smoother
    # - lifecycle managers
    #
    # Do not separately include navigation_launch.py because
    # bringup_launch.py includes it internally.
    # =========================================================

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_nav2_bringup,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'slam': 'False',
            'use_composition': 'False',
        }.items()
    )

    # Nav2 smoothing -> ultrasound slowdown -> transition smoothing -> motors.
    # The transition smoother publishes cmd_vel_safe to the wheel controller.
    collision_params = os.path.join(
        pkg_amr_mapping, 'config', 'collision_monitor.yaml')
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_params, {'use_sim_time': use_sim_time}],
    )
    slowdown_velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='slowdown_velocity_smoother',
        output='screen',
        parameters=[collision_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_slow'),
                    ('cmd_vel_smoothed', 'cmd_vel_safe')],
    )
    collision_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            # Activate the smoother before enabling upstream commands.
            'node_names': ['slowdown_velocity_smoother', 'collision_monitor'],
        }],
    )

    # =========================================================
    # Arbitrate between Nav2 and joystick teleop on the way to
    # wheel_control_node.
    #
    # Nav2's velocity_smoother publishes Twists on /cmd_vel, and
    # teleop_node publishes joystick Twists on /teleop/cmd_vel.
    # teleop_node only publishes while the stick is deflected, so
    # once it goes quiet for `timeout` seconds, twist_mux falls
    # through to Nav2's lower-priority topic. Without this, letting
    # both publish directly onto /diff_drive_controller/cmd_vel
    # caused Nav2's commands to be intermittently stomped by
    # whichever topic's message landed last, i.e. the
    # move-stop-move-stop behavior.
    # =========================================================

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            os.path.join(pkg_amr_mapping, 'config', 'twist_mux.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
    )

    # =========================================================
    # Trigger AMCL global localization on startup.
    #
    # The robot may be placed anywhere on the map, so a single
    # guessed initial pose is not reliable. This spreads AMCL's
    # particles across the whole map instead of a small area
    # around one point. The robot still needs a brief spin or
    # drive after launch so the particle filter can converge on
    # the correct pose using distinct laser scans.
    # =========================================================

    global_localization_trigger = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/reinitialize_global_localization',
                    'std_srvs/srv/Empty',
                ],
                output='screen',
            )
        ]
    )

    # =========================================================
    # Launch description
    # =========================================================

    return LaunchDescription([

        map_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,

        lidar_launch,
        # motor_launch,
        cmd_vel_bridge,
        scan_relay,
        ultrasound_center,
        ultrasound_right,
        ultrasound_left,
        range_filter,
        range_to_scan,

        wheel_control_node,

        # Transform tree
        laser_tf,
        depth_tf,
        us_center_tf,
        us_right_tf,
        us_left_tf,

        # Physical robot hardware




        # Visualization
        rviz_node,

        # Localization and navigation
        nav2_bringup,
        slowdown_velocity_smoother,
        collision_monitor,
        collision_lifecycle_manager,
        global_localization_trigger,

        # Manual control
        # joy_node,
        # teleop_node,
        camera_launch,
    ])