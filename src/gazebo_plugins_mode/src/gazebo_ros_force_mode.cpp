// Copyright 2013 Open Source Robotics Foundation, Inc.
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

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include "gazebo_plugins/gazebo_ros_force_mode.hpp"
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace gazebo_plugins
{
// 定义一个名为GazeboRosForcePrivate的私有辅助类，用于封装Gazebo ROS插件中与力施加相关的数据成员。

class GazeboRosForcePrivate
{
public:
  // 指向应用力的Gazebo物理link对象的指针。该link是力作用的目标对象。
  gazebo::physics::LinkPtr link_;

  // 指向GazeboROS节点的智能指针，用于和ROS系统进行通信交互。
  gazebo_ros::Node::SharedPtr ros_node_;

  // 订阅器，用于订阅geometry_msgs::msg::Wrench类型的消息，即接收从ROS系统传来的力矩数据。
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;

  // 存储当前插件对模型施加的力矩信息的数据容器，包含力和力矩两个分量。
  geometry_msgs::msg::Wrench wrench_msg_;

  // 指向Gazebo更新事件连接的指针，用于在仿真循环中实时更新并施加力的效果。
  gazebo::event::ConnectionPtr update_connection_;

  // 标记布尔值，指示力是否应在世界坐标系下应用，而不是link自身的坐标系。如果为true，则表示力将在世界坐标系下施加。
  bool force_on_world_frame_;
};

// GazeboRosForce 类的构造函数，初始化私有数据成员（PIMPL 模式）
GazeboRosForce::GazeboRosForce()
: impl_(std::make_unique<GazeboRosForcePrivate>())
{
    // 在此处可以添加额外的构造函数逻辑，如初始化 ROS 节点、订阅器等，但此处省略了具体的实现细节。
    // 因为在 PIMPL 模式下，私有实现部分已经通过 std::make_unique 创建并初始化了。
}

// GazeboRosForce 类的析构函数
GazeboRosForce::~GazeboRosForce()
{
    // 在此处通常不需要执行任何操作，因为 std::unique_ptr 会自动管理其指向的对象，在类实例被销毁时也会正确地释放私有实现部分占用的资源。
}

// GazeboRosForce 类的 Load 方法，用于加载模型并初始化相关参数

void GazeboRosForce::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // 获取 ROS 日志器
  auto logger = rclcpp::get_logger("gazebo_ros_force");

  // 获取目标link名称，如果未设置则报错并返回
  if (!sdf->HasElement("link_name")) {
    RCLCPP_ERROR(logger, "Force plugin missing <link_name>, cannot proceed");
    return;
  }

  // 获取 SDF 中指定的link名称
  auto link_name = sdf->GetElement("link_name")->Get<std::string>();

  // 获取指向目标link的指针，并检查link是否存在，不存在时报错并返回
  impl_->link_ = model->GetLink(link_name);
  if (!impl_->link_) {
    RCLCPP_ERROR(logger, "Link named: %s does not exist\n", link_name.c_str());
    return;
  }

  // 获取力作用坐标系（世界或link），若未设置，则默认为世界坐标系
  if (!sdf->HasElement("force_frame")) {
    RCLCPP_INFO(
      logger, "Force plugin missing <force_frame> wasn't set,"
      "therefore it's been set as 'world'. The other option is 'link'.");
    impl_->force_on_world_frame_ = true;
  } else {
    // 根据SDF配置中设定的坐标系进行相应处理
    auto force_frame = sdf->GetElement("force_frame")->Get<std::string>();
    if (force_frame == "world") {
      impl_->force_on_world_frame_ = true;
    } else if (force_frame == "link") {
      impl_->force_on_world_frame_ = false;
    } else {
      RCLCPP_ERROR(logger, "Force plugin <force_frame> can only be 'world' or 'link'");
      return;
    }
  }

  // 创建 ROS 节点实例并获取其 QoS 配置
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // 订阅 wrench 消息
  impl_->wrench_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Wrench>(
    "gazebo_ros_force", 
    qos.get_subscription_qos("gazebo_ros_force", rclcpp::SystemDefaultsQoS()),
    std::bind(&GazeboRosForce::OnRosWrenchMsg, this, std::placeholders::_1));

  // 注册一个在每次仿真迭代开始时调用的回调函数
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosForce::OnUpdate, this));
}

// GazeboRosForce 类的 OnRosWrenchMsg 回调函数，用于处理接收到的力矩消息

void GazeboRosForce::OnRosWrenchMsg(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
  // 将接收到的 ROS 力矩消息中的力分量赋值给私有成员变量 wrench_msg_
  impl_->wrench_msg_.force.x = msg->force.x;
  impl_->wrench_msg_.force.y = msg->force.y;
  impl_->wrench_msg_.force.z = msg->force.z;

  // 将接收到的 ROS 力矩消息中的扭矩分量也赋值给私有成员变量 wrench_msg_
  impl_->wrench_msg_.torque.x = msg->torque.x;
  impl_->wrench_msg_.torque.y = msg->torque.y;
  impl_->wrench_msg_.torque.z = msg->torque.z;

  // 这个回调函数在接收到新的力矩消息时被触发，并将消息内容存储到插件内部的 wrench_msg_ 中。
  // 在后续的仿真循环中，通过 OnUpdate 方法读取这个 wrench_msg_ 并施加相应的力和力矩到指定link上。
}
// GazeboRosForce 类的 OnUpdate 方法，在每次仿真迭代开始时被调用

void GazeboRosForce::OnUpdate()
{
#ifdef IGN_PROFILER_ENABLE
  // 如果启用了IGN_PROFILER，开始记录“GazeboRosForce::OnUpdate”区域的性能数据
  IGN_PROFILE("GazeboRosForce::OnUpdate");
  // 开始记录内部的“Aplly forces”区域的性能数据
  IGN_PROFILE_BEGIN("Aplly forces");
#endif

  // 根据force_on_world_frame_标志决定力和力矩是否在世界坐标系或link坐标系中施加
  if (impl_->force_on_world_frame_) {
    // 在世界坐标系下添加力
    impl_->link_->AddForce(
      gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_.force));
    // 在世界坐标系下添加扭矩
    impl_->link_->AddTorque(
      gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_.torque));
  } else {
    // 在link坐标系下添加相对力
    impl_->link_->AddRelativeForce(
      gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_.force));
    // 在link坐标系下添加相对扭矩
    impl_->link_->AddRelativeTorque(
      gazebo_ros::Convert<ignition::math::Vector3d>(impl_->wrench_msg_.torque));
  }

#ifdef IGN_PROFILER_ENABLE
  // 结束记录"Aplly forces"区域的性能数据
  IGN_PROFILE_END();
#endif
}

// 注册此插件到Gazebo系统中，使其可用
GZ_REGISTER_MODEL_PLUGIN(GazeboRosForce)
}  // namespace gazebo_plugins