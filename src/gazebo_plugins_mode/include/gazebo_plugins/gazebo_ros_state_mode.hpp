// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAZEBO_ROS__GAZEBO_ROS_STATE_HPP_
#define GAZEBO_ROS__GAZEBO_ROS_STATE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_ros
{
class GazeboRosStatePrivate;

/// \brief 提供服务和主题接口，用于查询和设置仿真环境中的实体（如模型或链接）的状态，包括位置和速度等信息。
///
/// 服务：
///
/// - get_entity_state (gazebo_msgs::srv::GetEntityState)
///   获取指定实体的位置和速度。
///
/// - set_entity_state (gazebo_msgs::srv::SetEntityState)
///   设置指定实体的位置和速度。

class GazeboRosState : public gazebo::WorldPlugin
{
public:
  /// \brief 构造函数
  GazeboRosState();

  /// \brief 析构函数
  virtual ~GazeboRosState();

  // \brief 继承自 WorldPlugin 的加载方法，在世界加载时调用以初始化插件
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

private:
  /// \brief 私有实现类的实例，用于封装内部实现细节
  std::unique_ptr<GazeboRosStatePrivate> impl_;
};

}  // namespace gazebo_ros

// 定义文件结束标签，防止重复包含
#endif  // GAZEBO_ROS__GAZEBO_ROS_STATE_HPP_
