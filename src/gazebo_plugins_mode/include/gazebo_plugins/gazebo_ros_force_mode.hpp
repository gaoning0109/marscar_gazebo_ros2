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
#include <geometry_msgs/msg/wrench.hpp>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosForcePrivate;

/**
 * \brief 本插件从ROS主题收集数据，并相应地将力矩施加到link上。
 *
 * \details 最后接收到的力会在每个仿真迭代过程中持续累加到链接上。发送一个空/零消息来停止施加力。

 * 示例用法：
 *
\code{.xml}
    <plugin name="gazebo_ros_force" filename="libgazebo_ros_force.so">

      <!-- 添加一个命名空间 -->
      <ros>
        <namespace>/test</namespace>

        <!-- 重映射默认主题 -->
        <remapping>gazebo_ros_force:=force_test</remapping>
      </ros>

      <!-- 指定模型内部将接收力的链接名称 -->
      <link_name>link</link_name>

      <!-- 力/力矩施加的坐标系（选项：world，即世界坐标系；link，即链接坐标系） -->
      <force_frame>link</force_frame>

    </plugin>
\endcode
 */


/**
 * \brief GazeboRosForce类是Gazebo模型插件，用于从ROS主题接收力矩数据并将其施加到模型的指定link上。
 */
class GazeboRosForce : public gazebo::ModelPlugin
{
public:
  /// \brief 构造函数
  GazeboRosForce();

  /// \brief 析构函数
  virtual ~GazeboRosForce();

  // \brief 继承自gazebo::ModelPlugin的加载方法，在模型加载时调用以初始化插件参数和连接 ROS 主题
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  /// \brief 可选回调函数，在每个仿真迭代过程中被调用，用于根据接收到的力矩消息更新模型状态
  virtual void OnUpdate();

private:
  /// \brief 当接收到一个ROS力矩消息时调用的回调函数
  /// \param[in] msg 新接收的表示要施加的新力矩的ROS消息
  void OnRosWrenchMsg(const geometry_msgs::msg::Wrench::SharedPtr msg);

  /// \brief 私有数据成员，采用PIMPL模式封装内部实现细节
  std::unique_ptr<GazeboRosForcePrivate> impl_;
};

// 
//该插件的主要功能是在仿真过程中根据接收到的ROS力矩消息对模型的某个链接施加力矩。
//在Load方法中会进行必要的初始化设置，包括与ROS节点建立连接、订阅力矩消息等。
//OnUpdate方法则会在每一步仿真迭代时被调用，检查是否有新的力矩消息，并据此更新链接上的力矩。
//OnRosWrenchMsg方法负责处理接收到的具体力矩消息内容。
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_FORCE_HPP_