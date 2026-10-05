-- Cartographer 2D mapping for a small differential-drive AMR with YDLIDAR.
--
-- Goals:
-- - Map faster with fewer passes / less revisit effort
-- - Still trust wheel odometry in feature-poor hallways
-- - Laser range: 0.25 m to 8.0 m (scan remapped to /scan_reliable)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",

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

  -- Publish submaps/poses more often for faster live map growth.
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ---------------------------------------------------------------------------
-- Laser / range data
-- ---------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.min_range = 0.25
TRAJECTORY_BUILDER_2D.max_range = 8.0
-- Longer free-space rays fill corridors/rooms in one pass.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Keep more points per scan so one drive-by builds a denser map.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.4
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 8.0

-- ---------------------------------------------------------------------------
-- Insert scans more densely (fewer gaps → fewer return passes)
-- ---------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(5.0)

-- ---------------------------------------------------------------------------
-- Real-time correlative scan matcher (odom-biased for hallways)
-- ---------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window =
    0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window =
    math.rad(20.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight =
    10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =
    40.0

-- ---------------------------------------------------------------------------
-- Ceres scan matcher (keep prior odom trust)
-- ---------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- ---------------------------------------------------------------------------
-- Submaps: finish faster + mark free/occupied sooner per observation
-- ---------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space =
    true
-- Slightly stronger updates so walls/free space settle with one pass.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability =
    0.60
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability =
    0.48

-- ---------------------------------------------------------------------------
-- Pose graph / loop closure
-- ---------------------------------------------------------------------------

POSE_GRAPH.optimize_every_n_nodes = 20

POSE_GRAPH.constraint_builder.min_score = 0.75
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.35
POSE_GRAPH.constraint_builder.log_matches = true

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
