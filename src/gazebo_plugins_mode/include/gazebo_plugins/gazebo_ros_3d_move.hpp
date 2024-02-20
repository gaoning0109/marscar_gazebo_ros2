// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE1_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE1_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRos3DMovePrivate;

/// \brief 提供三维空间移动功能的Gazebo模型插件，通过ROS接口控制模型在三维空间中的位置和姿态。
class GazeboRos3DMove : public gazebo::ModelPlugin
{
public:
  /// \brief 构造函数
  GazeboRos3DMove();

  /// \brief 析构函数
  ~GazeboRos3DMove();

protected:
  // \brief 继承自 ModelPlugin 的加载方法，在模型加载时调用以初始化插件
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  // \brief 继承自 ModelPlugin 的重置方法，在仿真重置时调用以重置插件状态
  void Reset() override;

private:
  /// \brief 私有数据成员，用于封装内部实现细节
  std::unique_ptr<GazeboRos3DMovePrivate> impl_;
};

}  // namespace gazebo_plugins

// 定义文件结束标签，防止重复包含
#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_3D_MOVE_HPP_