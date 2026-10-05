-- Cartographer 2D mapping configuration for a small differential-drive AMR
-- using a YDLIDAR.
--
-- Configuration goals:
-- - Trust wheel odometry in feature-poor hallways.
-- - Use lidar scan matching without allowing excessive longitudinal sliding.
-- - Use conservative loop closure.
-- - Laser range: 0.25 m to 8.0 m.
-- - Remap the scan topic to /scan_reliable in the launch file.


include "map_builder.lua"
include "trajectory_builder.lua"


options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",

  -- The robot's odometry system already publishes odom -> base_link.
  provide_odom_frame = false,

  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 1.0,

  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}


-- Enable the 2D trajectory builder.
MAP_BUILDER.use_trajectory_builder_2d = true


-- ---------------------------------------------------------------------------
-- Laser and range-data configuration
-- ---------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.min_range = 0.25

TRAJECTORY_BUILDER_2D.max_range = 8.0

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0

TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1


-- ---------------------------------------------------------------------------
-- Real-time correlative scan matcher
-- ---------------------------------------------------------------------------
-- The higher translation correction cost keeps the estimated pose close to
-- wheel odometry when the lidar cannot observe forward motion in a hallway.

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window =
    0.1

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window =
    math.rad(20.0)

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight =
    10.0  -- Tested value: 0.1; caused worse longitudinal sliding

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =
    40.0  -- Older value: 1e-1; numerically identical


-- ---------------------------------------------------------------------------
-- Ceres scan matcher
-- ---------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight =
    20.0  -- Tested value: 10.0; produced worse hallway results

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight =
    40.0  -- Older value: 40.0; unchanged


-- ---------------------------------------------------------------------------
-- Submap configuration
-- ---------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.submaps.num_range_data =
    35

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution =
    0.05


-- ---------------------------------------------------------------------------
-- Pose graph and loop-closure configuration
-- ---------------------------------------------------------------------------

POSE_GRAPH.optimize_every_n_nodes =
    35

POSE_GRAPH.constraint_builder.min_score =
    0.8

POSE_GRAPH.constraint_builder.global_localization_min_score =
    0.70

POSE_GRAPH.constraint_builder.max_constraint_distance =
    15.0

-- Logs accepted and rejected loop-closure scan matches.
POSE_GRAPH.constraint_builder.log_matches =
    true  -- Older configuration: not explicitly set

POSE_GRAPH.optimization_problem.huber_scale =
    1e2


-- ---------------------------------------------------------------------------
-- Local SLAM and wheel-odometry weights
-- ---------------------------------------------------------------------------
-- High odometry weights are retained because forward displacement is poorly
-- observable from lidar scans containing only two long, parallel walls.

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight =
    1e5  -- Older configuration: inherited the same default value

POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight =
    1e5  -- Older configuration: inherited the same default value

POSE_GRAPH.optimization_problem.odometry_translation_weight =
    1e5  -- Tested value: 1e4; caused more longitudinal sliding

POSE_GRAPH.optimization_problem.odometry_rotation_weight =
    1e5  -- Tested value: 1e3; caused weaker heading stability


return options