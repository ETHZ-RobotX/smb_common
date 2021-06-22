-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

----------
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4



POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.optimize_every_n_nodes = 150   


----loop closure
--constraint builder
POSE_GRAPH.constraint_builder.max_constraint_distance = 100.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 8.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 5.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.global_sampling_ratio = 0.02

----non-loop-closure scan matcher
POSE_GRAPH.constraint_builder.min_score = 0.6--0.42
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

----optimization problem
POSE_GRAPH.optimization_problem.rotation_weight = 3e2 --Scaling parameter for the IMU rotation term.
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 12
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = true

--
MAX_3D_RANGE = 20


----scan matching
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.15   --Resolution of the ‘high_resolution’ map in meters used for local SLAM and loop closure.
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = MAX_3D_RANGE - 5.  --Maximum range to filter the point cloud to before insertion into the ‘high_resolution’ map.
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.35   --Resolution of the ‘low_resolution’ version of the map in meters used for local SLAM only.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 2.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight =5.0
TRAJECTORY_BUILDER_3D.min_range = 2   --Rangefinder points outside these ranges will be dropped.
TRAJECTORY_BUILDER_3D.max_range = MAX_3D_RANGE  --Rangefinder points outside these ranges will be dropped.
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1  --Voxel filter that gets applied to the range data immediately after cropping.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = MAX_3D_RANGE

return options
