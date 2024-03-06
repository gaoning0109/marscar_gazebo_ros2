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
#include <gazebo/physics/World.hh>
#include "gazebo_plugins/gazebo_ros_test.hpp"
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
      void OnUpdate(const gazebo::common::UpdateInfo & _info);
       void OnJoy(const sensor_msgs::msg::Joy::SharedPtr _msg);
      // 指向应用力的Gazebo物理link对象的指针。该link是力作用的目标对象。
      gazebo::physics::LinkPtr link_;
      gazebo::physics::JointPtr joint_;
      // 指向GazeboROS节点的智能指针，用于和ROS系统进行通信交互。
      gazebo_ros::Node::SharedPtr ros_node_;
            // 保护在回调中访问的变量
            std::mutex lock_;
        gazebo::physics::WorldPtr world_;
      // 订阅器，用于订阅geometry_msgs::msg::Wrench类型的消息，即接收从ROS系统传来的力矩数据。
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      // 发布里程计信息的发布者
      rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr test_pub_;
      // 接收到的目标速度命令
      sensor_msgs::msg::Joy joy_msg_;
      // 存储最新的里程计消息
      geometry_msgs::msg::Pose data_;
      // 指向Gazebo更新事件连接的指针，用于在仿真循环中实时更新并施加力的效果。
      gazebo::event::ConnectionPtr update_connection_;
            // 指向模型实例的指针
            gazebo::physics::ModelPtr model_;
      // 是否发布里程计消息的标志位
      bool publish_topic_;
    /// \brief 更新周期，单位为秒.
    double update_period_;

    /// \brief 上次更新时间.
    gazebo::common::Time last_update_time_;
};

// GazeboRosTest 类的构造函数，初始化私有数据成员（PIMPL 模式）
GazeboRosTest::GazeboRosTest()
: impl_(std::make_unique<GazeboRosTestPrivate>())
{
}

// GazeboRosTest 类的析构函数
GazeboRosTest::~GazeboRosTest()
{
}

// GazeboRosTest 类的 Load 方法，用于加载模型并初始化相关参数

void GazeboRosTest::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // 获取 ROS 日志器
  auto logger = rclcpp::get_logger("gazebo_ros_test");
        impl_->model_ = model;
          impl_->world_ = model->GetWorld();
  // 获取目标link名称，如果未设置则报错并返回
  if (!sdf->HasElement("link_name")) {
    RCLCPP_ERROR(logger, " plugin missing <link_name>, cannot proceed");
    return;
  }
  if (!sdf->HasElement("joint_name")) {
    RCLCPP_ERROR(logger, " plugin missing <joint_name>, cannot proceed");
    return;
  }
  // 获取 SDF 中指定的link名称
  auto link_name = sdf->GetElement("link_name")->Get<std::string>();
  // 获取 SDF 中指定的link名称
  auto joint_name = sdf->GetElement("joint_name")->Get<std::string>();

  // 获取指向目标link的指针，并检查link是否存在，不存在时报错并返回
  impl_->link_ = model->GetLink(link_name);
  if (!impl_->link_) {
    RCLCPP_ERROR(logger, "Link named: %s does not exist\n", link_name.c_str());
    return;
  }
  impl_->joint_ = model->GetJoint(joint_name);
  if (!impl_->joint_) {
    RCLCPP_ERROR(logger, "Joint named: %s does not exist\n", joint_name.c_str());
    return;
  }

  // 创建 ROS 节点实例并获取其 QoS 配置
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();
  impl_->joy_sub_ = impl_->ros_node_->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 
    qos.get_subscription_qos("joy", rclcpp::SystemDefaultsQoS()),
    std::bind(&GazeboRosTestPrivate::OnJoy,impl_.get(), std::placeholders::_1));

  impl_->publish_topic_ = sdf->Get<bool>("publish_topic", false).first;
  if (impl_->publish_topic_) {
    impl_->test_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::Pose>(
      "gazebo_pub", qos.get_publisher_qos("gazebo_pub", rclcpp::QoS(1)));
      //qos.get_publisher_qos("odom", rclcpp::QoS(1))：
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise topic on [%s]",
      impl_->test_pub_->get_topic_name());
  }
  // 注册一个在每次仿真迭代开始时调用的回调函数 gazebo::event::Events：Gazebo提供了一系列事件接口，
  //其中Events类封装了所有的事件类型及其相关操作。在这里，调用ConnectWorldUpdateBegin方法是为了监听仿真世界的"开始更新"事件。

  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosTestPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
  auto update_rate = sdf->Get<double>("update_rate", 100.0).first;
  
  // 根据更新率计算更新周期（以秒为单位），如果更新率为正则进行计算，否则设为0
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  // 初始化最后一次更新时间，将其设置为当前仿真世界的模拟时间
  impl_->last_update_time_ = impl_->world_->SimTime();
}

void GazeboRosTestPrivate::OnJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{       
          // 将接收到的 ROS 力矩消息中的力分量赋值给私有成员变量 joy_msg_
          joy_msg_ = *msg;
}
// GazeboRosTest 类的 OnUpdate 方法，在每次仿真迭代开始时被调用

void GazeboRosTestPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{  
    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

      // 如果时间间隔小于预定的更新周期，则返回，不进行数据更新与发布
      if (seconds_since_last_update < update_period_) {
        return;
      }
        ignition::math::Pose3d model_pose = link_->WorldPose();
        data_ = gazebo_ros::Convert<geometry_msgs::msg::Pose>(model_pose);
          // 当时间差大于设定的更新周期时，执行以下操作

       //         model_->SetLinearVel(
        //         ignition::math::Vector3d(
        //             joy_msg_.axes[0],
        //             joy_msg_.axes[1],
        //             joy_msg_.axes[4])); 
        std::cout<<joy_msg_.axes[4]<<std::endl;
        // joint_->SetVelocity(0,joy_msg_.axes[0]*10);
        joint_->SetForce(0,joy_msg_.axes[0]*100);


        // link_->SetLinearVel(
        // ignition::math::Vector3d(
        //     joy_msg_.axes[0],
        //     joy_msg_.axes[1],
        //     joy_msg_.axes[4]));

        // link_->AddLinkForce(
        // ignition::math::Vector3d(
        //     joy_msg_.axes[0]*100,
        //     joy_msg_.axes[1]*100,
        //     joy_msg_.axes[4]*100));
        // link_->SetForce(
        // ignition::math::Vector3d(
        //     joy_msg_.axes[0]*100,
        //     joy_msg_.axes[1]*100,
        //     joy_msg_.axes[4]*100));

      // Publish
      //gazebo_ros::Convert<geometry_msgs::msg::Pose>(model_pose) 是在 Gazebo 和 ROS 环境中进行数据类型转换的一个示例。
      //gazebo_ros 提供了一系列转换函数，用于将 Gazebo 的内部数据类型与 ROS 消息类型之间相互转换。<geometry_msgs::msg::Pose> 是模板参数，
      //表示要转换的目标类型是 ROS 中的 geometry_msgs::msg::Pose 消息类型，该消息通常用于描述三维空间中的位置和姿态（位姿）信息。
        test_pub_->publish(data_);
  last_update_time_ = _info.simTime;
}

// 注册此插件到Gazebo系统中，使其可用
GZ_REGISTER_MODEL_PLUGIN(GazeboRosTest)
}  // namespace gazebo_plugins