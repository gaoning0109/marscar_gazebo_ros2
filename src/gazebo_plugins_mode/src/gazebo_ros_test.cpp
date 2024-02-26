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
#include "gazebo_plugins/gazebo_ros_test.hpp"
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <string>

namespace gazebo_plugins
{
// 定义一个名为GazeboRosTestPrivate的私有辅助类，用于封装Gazebo ROS插件中与力施加相关的数据成员。

class GazeboRosTestPrivate
{
public:
      void OnUpdate();
       void OnJoy(const sensor_msgs::msg::Joy::SharedPtr _msg);
      // 指向应用力的Gazebo物理link对象的指针。该link是力作用的目标对象。
      gazebo::physics::LinkPtr link_;

      // 指向GazeboROS节点的智能指针，用于和ROS系统进行通信交互。
      gazebo_ros::Node::SharedPtr ros_node_;
            // 保护在回调中访问的变量
            std::mutex lock_;
      // 订阅器，用于订阅geometry_msgs::msg::Wrench类型的消息，即接收从ROS系统传来的力矩数据。
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      // 发布里程计信息的发布者
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
      // 接收到的目标速度命令
      sensor_msgs::msg::Joy joy_msg_;
      // 存储最新的里程计消息
      nav_msgs::msg::Odometry odom_;
      // 指向Gazebo更新事件连接的指针，用于在仿真循环中实时更新并施加力的效果。
      gazebo::event::ConnectionPtr update_connection_;
            // 指向模型实例的指针
            gazebo::physics::ModelPtr model_;
      // 是否发布里程计消息的标志位
      bool publish_odom_;
};

// GazeboRosTest 类的构造函数，初始化私有数据成员（PIMPL 模式）
GazeboRosTest::GazeboRosTest()
: impl_(std::make_unique<GazeboRosTestPrivate>())
{
    // 在此处可以添加额外的构造函数逻辑，如初始化 ROS 节点、订阅器等，但此处省略了具体的实现细节。
    // 因为在 PIMPL 模式下，私有实现部分已经通过 std::make_unique 创建并初始化了。
}

// GazeboRosTest 类的析构函数
GazeboRosTest::~GazeboRosTest()
{
    // 在此处通常不需要执行任何操作，因为 std::unique_ptr 会自动管理其指向的对象，在类实例被销毁时也会正确地释放私有实现部分占用的资源。
}

// GazeboRosTest 类的 Load 方法，用于加载模型并初始化相关参数

void GazeboRosTest::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // 获取 ROS 日志器
  auto logger = rclcpp::get_logger("gazebo_ros_force");
        impl_->model_ = model;
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


  // 创建 ROS 节点实例并获取其 QoS 配置
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->joy_sub_ = impl_->ros_node_->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 
    qos.get_subscription_qos("joy", rclcpp::SystemDefaultsQoS()),
    std::bind(&GazeboRosTestPrivate::OnJoy,impl_.get(), std::placeholders::_1));
  // Advertise odometry topic
  impl_->publish_odom_ = sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }
  // 注册一个在每次仿真迭代开始时调用的回调函数
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosTestPrivate::OnUpdate, impl_.get()));
}

void GazeboRosTestPrivate::OnJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{        std::lock_guard<std::mutex> scoped_lock(std::mutex lock_);
  // 将接收到的 ROS 力矩消息中的力分量赋值给私有成员变量 wrench_msg_
  joy_msg_ = *msg;
        model_->SetLinearVel(
        ignition::math::Vector3d(
            joy_msg_.axes[0],
            joy_msg_.axes[1],
            joy_msg_.axes[4])); 
std::cout<<joy_msg_.axes[4]<<std::endl;
}
// GazeboRosTest 类的 OnUpdate 方法，在每次仿真迭代开始时被调用

void GazeboRosTestPrivate::OnUpdate()
{
      // 使用std::lock_guard确保线程安全
    std::lock_guard<std::mutex> scoped_lock(std::mutex lock_);
    ignition::math::Pose3d model_pose = link_->WorldPose();
    odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(model_pose);
  // Publish
    odometry_pub_->publish(odom_);

}

// 注册此插件到Gazebo系统中，使其可用
GZ_REGISTER_MODEL_PLUGIN(GazeboRosTest)
}  // namespace gazebo_plugins