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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_

#include <gazebo/common/Plugin.hh>

// For std::unique_ptr, could be removed
#include <memory>

namespace gazebo_plugins
{
// 前向声明私有数据类
class GazeboRosTemplatePrivate;

/// \brief 示例ROS驱动的Gazebo插件，包含了一些有用的样板代码。
///
/// \details 该插件是一个`ModelPlugin`类型，但根据需要可以是任何支持的Gazebo插件类型，如System、Visual、GUI、World、Sensor等。
class GazeboRosTemplate : public gazebo::ModelPlugin
{
public:
  /// \brief 构造函数
  GazeboRosTemplate();

  /// \brief 析构函数
  virtual ~GazeboRosTemplate();

  /// \brief 当插件被加载时，Gazebo会调用此方法。
  /// \param[in] model 指向父模型的指针。对于其他类型的插件，可能会暴露出不同的实体，例如 `gazebo::sensors::SensorPtr`，`gazebo::physics::WorldPtr`，`gazebo::rendering::VisualPtr` 等。
  /// \param[in] sdf 包含用户定义参数的SDF元素。
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
  /// \brief 可选回调，在每次仿真迭代时调用。
  virtual void OnUpdate();

private:
  /// \brief 推荐使用的PIMPL模式。这个变量应持有所有的私有数据成员。
  std::unique_ptr<GazeboRosTemplatePrivate> impl_;
};

}  // namespace gazebo_plugins

// 定义文件结束标签，防止重复包含
#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_TEMPLATE_HPP_