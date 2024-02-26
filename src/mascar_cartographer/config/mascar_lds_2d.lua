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

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

-- 引入 "map_builder.lua" 和 "trajectory_builder.lua" 脚本中的配置模块

options = { -- 配置参数列表
  -- 指定地图构建器和轨迹构建器模块
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 设置坐标系与帧ID
  map_frame = "map",
  tracking_frame = "Mars6Mars_Rover__Curiosity_1_0__-_whole_body_link",
  published_frame = "odom",
  odom_frame = "odom",
  
  -- 控制是否提供 odom 帧以及投影到二维平面
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  
  -- 输入源设置
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  
  -- 扫描仪和点云参数
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- 时间超时及发布周期设定
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- 各种传感器数据采样率
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 设置使用 2D 轨迹构建器
MAP_BUILDER.use_trajectory_builder_2d = true

-- 针对 2D 轨迹构建器的特定参数调整
TRAJECTORY_BUILDER_2D.min_range = 0.12 -- 最小有效测量范围
TRAJECTORY_BUILDER_2D.max_range = 3.5   -- 最大有效测量范围
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3. -- 缺失数据时的射线长度
TRAJECTORY_BUILDER_2D.use_imu_data = false     -- 不使用 IMU 数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- 使用在线相关扫描匹配
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- 运动滤波器最大角度限制（以弧度为单位）

-- 定义位姿图约束构建器的参数
POSE_GRAPH.constraint_builder.min_score = 0.65 -- 最小约束得分阈值
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7 -- 全局定位最小得分阈值

-- 控制每多少个节点进行优化
-- POSE_GRAPH.optimize_every_n_nodes = 0

-- 返回整个配置选项集
return options