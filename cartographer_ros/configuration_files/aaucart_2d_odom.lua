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
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1, 
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.1,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Local SLAM

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 5

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 15
TRAJECTORY_BUILDER_2D.submaps.resolution = 0.08


--GLobal SLAM

-- POSE_GRAPH.optimize_every_n_nodes = 10

--POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10 
--POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 10
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 10
POSE_GRAPH.optimization_problem.odometry_translation_weight = 6e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 4e4


return options



-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight =40
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight =20


-- TRAJECTORY_BUILDER_3D.use_imu_data = true

-- TRAJECTORY_BUILDER_2D = {
--   -- use_imu_data = false,
--   -- min_range = 0.5,
--   -- max_range = 20.,
--   -- min_z = -0.8,
--   -- max_z = 1.,
--   -- missing_data_ray_length = 0.,
--   num_accumulated_range_data = 50,
--   -- voxel_filter_size = 0.1,

--   -- adaptive_voxel_filter = {
--   --   max_length = 0.5,
--   --   min_num_points = 200,
--   --   max_range = 50.,
--   -- },

--   -- loop_closure_adaptive_voxel_filter = {
--   --   max_length = 0.5,
--   --   min_num_points = 100,
--   --   max_range = 50.,
--   -- },

--   -- not neede if the sensor readings from odometry and other sources are reasonably well
--   -- use_online_correlative_scan_matching = false,
--   -- real_time_correlative_scan_matcher = {
--   --   linear_search_window = 0.1,
--   --   angular_search_window = math.rad(20.),
--   --   translation_delta_cost_weight = 1e-1,
--   --   rotation_delta_cost_weight = 1e-1,
--   -- },

--   ceres_scan_matcher = {
--   --   occupied_space_weight = 1.,
--     translation_weight =40,
--     rotation_weight =20,
--   --   ceres_solver_options = {
--   --     use_nonmonotonic_steps = false,
--   --     max_num_iterations = 20,
--   --     num_threads = 4,
--   --   },
--   },

--   -- motion_filter = {
--   --   max_time_seconds = 5.,
--   --   max_distance_meters = 0.2,
--   --   max_angle_radians = math.rad(1.),
--   -- },

--   -- imu_gravity_time_constant = 10.,

--   submaps = {
--     resolution = 0.05,
--     num_range_data = 90,
--   --   range_data_inserter = {
--   --     insert_free_space = true,
--   --     hit_probability = 0.62,
--   --     miss_probability = 0.48,
--   --   },
--   },
-- }

-- POSE_GRAPH = {
--   -- optimize_every_n_nodes = 35,
--   -- constraint_builder = {

--   --   sampling_ratio = 0.3,
--   --   max_constraint_distance = 15.,
--   --   min_score = 0.55,
--   --   global_localization_min_score = 0.6,
--   --   loop_closure_translation_weight = 1, --1.1e4
--   --   loop_closure_rotation_weight = 1, --1e5
--   --   log_matches = true,

--   --   -- fast_correlative_scan_matcher = {
--   --   --   linear_search_window = 7.,
--   --   --   angular_search_window = math.rad(30.),
--   --   --   branch_and_bound_depth = 7,
--   --   -- },

--   --   -- ceres_scan_matcher = {
--   --   --   occupied_space_weight = 20.,
--   --   --   translation_weight = 10.,
--   --   --   rotation_weight = 1.,
--   --   --   ceres_solver_options = {
--   --   --     use_nonmonotonic_steps = true,
--   --   --     max_num_iterations = 10,
--   --   --     num_threads = 1,
--   --   --   },

--   -- --   },
--   -- },

--   -- matcher_translation_weight = 5e2,
--   -- matcher_rotation_weight = 1.6e3,

--   optimization_problem = {
--     -- huber_scale = 1e2,
--     -- acceleration_weight = 1e3,
--     -- rotation_weight = 3e5,
--     -- consecutive_node_translation_weight = 1e5,
--     -- consecutive_node_rotation_weight = 1e5,
--     -- fixed_frame_pose_translation_weight = 1, --1e1
--     -- fixed_frame_pose_rotation_weight = 1, --1e2
--     local_slam_pose_translation_weight = 0,
--     local_slam_pose_rotation_weight = 0,
--     odometry_translation_weight = 1e10,
--     odometry_rotation_weight = 1e10,
--     -- log_solver_summary = false,
--     -- ceres_solver_options = {
--     --   use_nonmonotonic_steps = false,
--     --   max_num_iterations = 50,
--     --   num_threads = 7,
--     -- },
--   },

--   -- max_num_final_iterations = 200,
--   -- global_sampling_ratio = 0.003,
--   -- log_residual_histograms = true,
--   -- global_constraint_search_after_n_seconds = 10.,

-- }

