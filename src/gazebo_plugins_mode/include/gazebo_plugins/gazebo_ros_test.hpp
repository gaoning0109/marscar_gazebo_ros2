// Copyright 2012 Open Source Robotics Foundation, Inc.
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_FORCE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_FORCE_HPP_

#include <gazebo/common/Plugin.hh>
#include <sensor_msgs/msg/joy.hpp>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosTestPrivate;

/**
 * \brief GazeboRosTest类是Gazebo模型插件
 */
class GazeboRosTest : public gazebo::ModelPlugin
{
public:
  /// \brief 构造函数
  GazeboRosTest();

  /// \brief 析构函数
  virtual ~GazeboRosTest();

  // \brief 继承自gazebo::ModelPlugin的加载方法，在模型加载时调用以初始化插件参数和连接 ROS 主题
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// \brief 可选回调函数，在每个仿真迭代过程中被调用，用于根据接收到的力矩消息更新模型状态
  // virtual void OnUpdate();

private:


  /// \brief 私有数据成员，采用PIMPL模式封装内部实现细节
  std::unique_ptr<GazeboRosTestPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_FORCE_HPP_