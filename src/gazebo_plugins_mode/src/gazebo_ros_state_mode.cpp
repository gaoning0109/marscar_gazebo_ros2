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

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Light.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_msgs/msg/link_states.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <gazebo_msgs/srv/get_entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>
#include <gazebo_ros/node.hpp>

#include <memory>

#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"
#include "gazebo_plugins/gazebo_ros_state_mode.hpp"

namespace gazebo_ros
{

class GazeboRosStatePrivate
{
public:
// 公开类成员函数

/// \brief 当世界（仿真环境）更新时的回调函数.
/// \param[in] _info 更新后的仿真信息.
void OnUpdate(const gazebo::common::UpdateInfo & _info);

/// \brief 获取实体状态服务的回调函数.
/// \param[in] req 请求消息指针
/// \param[out] res 响应消息指针
void GetEntityState(
  gazebo_msgs::srv::GetEntityState::Request::SharedPtr _req,
  gazebo_msgs::srv::GetEntityState::Response::SharedPtr _res);

/// \brief 设置实体状态服务的回调函数.
/// \param[in] req 请求消息指针
/// \param[out] res 响应消息指针
void SetEntityState(
  gazebo_msgs::srv::SetEntityState::Request::SharedPtr _req,
  gazebo_msgs::srv::SetEntityState::Response::SharedPtr _res);

/// \brief Gazebo中存储的世界指针，用于访问仿真环境.
gazebo::physics::WorldPtr world_;

/// \brief 用于ROS通信的节点对象，由gazebo_ros库管理.
gazebo_ros::Node::SharedPtr ros_node_;

/// \brief 处理获取实体状态请求的ROS服务.
rclcpp::Service<gazebo_msgs::srv::GetEntityState>::SharedPtr get_entity_state_service_;

/// \brief 处理设置实体状态请求的ROS服务.
rclcpp::Service<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_service_;

/// \brief 发布模型状态的ROS发布器.
rclcpp::Publisher<gazebo_msgs::msg::ModelStates>::SharedPtr model_states_pub_;

/// \brief 发布链接状态的ROS发布器.
rclcpp::Publisher<gazebo_msgs::msg::LinkStates>::SharedPtr link_states_pub_;

/// \brief 连接至世界更新事件，在每次迭代时被调用.
gazebo::event::ConnectionPtr world_update_event_;

/// \brief 更新周期，单位为秒.
double update_period_;

/// \brief 上次更新时间.
gazebo::common::Time last_update_time_;
};

// GazeboRosState类的构造函数
GazeboRosState::GazeboRosState()
: impl_(std::make_unique<GazeboRosStatePrivate>())
{
    // 使用C++14中的std::make_unique创建一个GazeboRosStatePrivate对象实例（实现PIMPL设计模式）
    // 这样可以隐藏类的私有实现细节，提高接口与实现的解耦性
}

// GazeboRosState类的析构函数
GazeboRosState::~GazeboRosState()
{
    // 当GazeboRosState对象销毁时，其内部持有的unique_ptr（impl_）也会自动销毁，
    // 因此会正确地清理并释放与GazeboRosStatePrivate相关的所有资源
}

// GazeboRosState类的Load方法，用于加载插件并初始化相关参数和ROS服务

void GazeboRosState::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // 将_world赋值给私有成员变量impl_->world_
  impl_->world_ = _world;

  // 使用_sdf创建并获取一个Gazebo ROS节点对象
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // 创建一个获取实体状态的服务，并绑定私有实现类中对应的回调函数
  impl_->get_entity_state_service_ = impl_->ros_node_->create_service<gazebo_msgs::srv::GetEntityState>(
    "get_entity_state", 
    std::bind(
      &GazeboRosStatePrivate::GetEntityState, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  // 创建一个设置实体状态的服务，并绑定私有实现类中对应的回调函数
  impl_->set_entity_state_service_ = impl_->ros_node_->create_service<gazebo_msgs::srv::SetEntityState>(
    "set_entity_state", 
    std::bind(
      &GazeboRosStatePrivate::SetEntityState, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));

  // 创建一个发布模型状态的主题，使用QoS策略保持最后一条消息
  impl_->model_states_pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::ModelStates>(
    "model_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  // 输出日志信息，表明正在发布的模型状态主题名称
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Publishing states of gazebo models at [%s]",
    impl_->model_states_pub_->get_topic_name());

  // 创建一个发布链接状态的主题，同样使用QoS策略保持最后一条消息
  impl_->link_states_pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::LinkStates>(
    "link_states", rclcpp::QoS(rclcpp::KeepLast(1)));

  // 输出日志信息，表明正在发布的链接状态主题名称
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Publishing states of gazebo links at [%s]",
    impl_->link_states_pub_->get_topic_name());

  // 注册世界更新开始事件的回调函数
  impl_->world_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosStatePrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  // 获取SDF中的更新率参数（默认为100Hz）
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  
  // 根据更新率计算更新周期（以秒为单位），如果更新率为正则进行计算，否则设为0
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  // 初始化最后一次更新时间，将其设置为当前仿真世界的模拟时间
  impl_->last_update_time_ = _world->SimTime();
}

// GazeboRosStatePrivate类的OnUpdate方法，用于在仿真世界更新时收集并发布模型和link的状态信息

void GazeboRosStatePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  // 计算自上次更新以来的时间（单位：秒）
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  // 如果时间间隔小于预定的更新周期，则返回，不进行数据更新与发布
  if (seconds_since_last_update < update_period_) {
    return;
  }

  // 初始化待发布的ModelStates和LinkStates消息
  gazebo_msgs::msg::ModelStates model_states;
  gazebo_msgs::msg::LinkStates link_states;

  // 遍历世界中的所有模型，并填充model_states消息
  for (const auto & model : world_->Models()) {
    // 将Gazebo Pose转换为ROS Pose并添加到model_states的消息中
    auto pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(model->WorldPose());
    model_states.pose.push_back(pose);
    model_states.name.push_back(model->GetName());

    // 获取模型的世界线速度和角速度，并转换为ROS Twist，添加到model_states的消息中
    geometry_msgs::msg::Twist twist;
    twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(model->WorldLinearVel());
    twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(model->WorldAngularVel());
    model_states.twist.push_back(twist);

    // 遍历模型的所有链接，并填充link_states消息
    for (unsigned int j = 0; j < model->GetChildCount(); ++j) {
      auto link = boost::dynamic_pointer_cast<gazebo::physics::Link>(model->GetChild(j));

      // 检查是否为有效的链接对象
      if (link) {
        // 添加链接名称到link_states的消息中
        link_states.name.push_back(link->GetScopedName());

        // 将Gazebo Link的位姿转换为ROS Pose，并添加到link_states的消息中
        pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(link->WorldPose());
        link_states.pose.push_back(pose);

        // 获取链接的世界线速度和角速度，并转换为ROS Twist，添加到link_states的消息中
        twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link->WorldLinearVel());
        twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(link->WorldAngularVel());
        link_states.twist.push_back(twist);
      }
    }
  }

  // 发布填充好的model_states和link_states消息
  model_states_pub_->publish(model_states);
  link_states_pub_->publish(link_states);

  // 更新最后更新时间戳
  last_update_time_ = _info.simTime;
}

// GazeboRosStatePrivate类的GetEntityState服务回调函数，用于获取实体的状态

void GazeboRosStatePrivate::GetEntityState(
  gazebo_msgs::srv::GetEntityState::Request::SharedPtr _req,
  gazebo_msgs::srv::GetEntityState::Response::SharedPtr _res)
{
  // 获取请求的目标实体
  auto entity = world_->EntityByName(_req->name);
  if (!entity) { // 如果找不到该实体
    _res->success = false; // 设置响应中的成功标志为false

    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "GetEntityState: entity [%s] does not exist", _req->name.c_str()); // 输出错误日志信息

    return; // 结束函数执行
  }

  // 获取实体在世界坐标系下的位姿、线速度和角速度
  auto entity_pose = entity->WorldPose();
  auto entity_lin_vel = entity->WorldLinearVel();
  auto entity_ang_vel = entity->WorldAngularVel();

  // 获取参考坐标系实体（如果存在）
  auto frame = world_->EntityByName(_req->reference_frame);
  if (frame) {
    // 计算相对于参考坐标系的位姿、线速度和角速度
    auto frame_pose = frame->WorldPose();
    auto frame_lin_vel = frame->WorldLinearVel();
    auto frame_ang_vel = frame->WorldAngularVel();

    entity_pose = entity_pose - frame_pose; // 计算相对位姿

    // 转换到相对于参考坐标系的速度
    entity_lin_vel = frame_pose.Rot().RotateVectorReverse(entity_lin_vel - frame_lin_vel);
    entity_ang_vel = frame_pose.Rot().RotateVectorReverse(entity_ang_vel - frame_ang_vel);
  } else if (_req->reference_frame == "" || _req->reference_frame == "world") {
    RCLCPP_DEBUG(
      ros_node_->get_logger(),
      "GetEntityState: reference_frame is empty/world, using inertial frame"); // 使用惯性坐标系作为默认参考系
  } else {
    _res->success = false;

    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "GetEntityState: reference entity [%s] not found, did you forget to scope the entity name?",
      _req->name.c_str()); // 输出错误日志信息

    return; // 结束函数执行
  }

  // 填充响应消息内容
  _res->header.stamp = Convert<builtin_interfaces::msg::Time>(world_->SimTime()); // 设置时间戳
  _res->header.frame_id = _req->reference_frame; // 设置参考坐标系ID
  _res->state.pose.position = Convert<geometry_msgs::msg::Point>(entity_pose.Pos()); // 设置位置信息
  _res->state.pose.orientation = Convert<geometry_msgs::msg::Quaternion>(entity_pose.Rot()); // 设置姿态信息
  _res->state.twist.linear = Convert<geometry_msgs::msg::Vector3>(entity_lin_vel); // 设置线速度信息
  _res->state.twist.angular = Convert<geometry_msgs::msg::Vector3>(entity_ang_vel); // 设置角速度信息
  _res->success = true; // 设置响应中的成功标志为true
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GazeboRosStatePrivate类的SetEntityState服务回调函数，用于设置实体的状态

void GazeboRosStatePrivate::SetEntityState(
  gazebo_msgs::srv::SetEntityState::Request::SharedPtr _req,
  gazebo_msgs::srv::SetEntityState::Response::SharedPtr _res)
{
  // 将ROS消息中的位置转换为Ignition Math Vector3d类型
  auto entity_pos = Convert<ignition::math::Vector3d>(_req->state.pose.position);
  
  // 将ROS消息中的姿态转换为Ignition Math Quaterniond类型，并规范化（避免零向量）
  auto entity_rot = Convert<ignition::math::Quaterniond>(_req->state.pose.orientation);
  entity_rot.Normalize();

  // 创建一个Ignition Math Pose3d对象表示实体的位置和姿态
  ignition::math::Pose3d entity_pose(entity_pos, entity_rot);

  // 转换线速度和角速度
  auto entity_lin_vel = Convert<ignition::math::Vector3d>(_req->state.twist.linear);
  auto entity_ang_vel = Convert<ignition::math::Vector3d>(_req->state.twist.angular);

  // 获取目标实体
  auto entity = world_->EntityByName(_req->state.name);
  if (!entity) {
    _res->success = false;

    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "SetEntityState: entity [%s] does not exist", _req->state.name.c_str());

    return;
  }

  // 获取参考坐标系实体（如果存在）
  auto frame = world_->EntityByName(_req->state.reference_frame);
  if (frame) {
    // 计算相对于世界坐标的实体位姿，并将速度转换到世界坐标系下
    auto frame_pose = frame->WorldPose();
    entity_pose = entity_pose + frame_pose;
    entity_lin_vel = frame_pose.Rot().RotateVector(entity_lin_vel);
    entity_ang_vel = frame_pose.Rot().RotateVector(entity_ang_vel);
  } else if (_req->state.reference_frame == "" || _req->state.reference_frame == "world") {
    RCLCPP_DEBUG(
      ros_node_->get_logger(),
      "SetEntityState: reference_frame is empty/world, using inertial frame");
  } else {
    _res->success = false;

    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "SetEntityState: reference entity [%s] not found, did you forget to scope the entity name?",
      _req->state.name.c_str());

    return;
  }

  // 设置实体状态
  auto model = boost::dynamic_pointer_cast<gazebo::physics::Model>(entity);
  auto link = boost::dynamic_pointer_cast<gazebo::physics::Link>(entity);
  auto light = boost::dynamic_pointer_cast<gazebo::physics::Light>(entity);
  //类型转换

  entity->SetWorldPose(entity_pose);
  // world_->SetPaused(is_paused);

  // 根据实体类型设置其速度信息
  if (model) {
    model->SetLinearVel(entity_lin_vel);
    model->SetAngularVel(entity_ang_vel);
  } else if (link) {
    link->SetLinearVel(entity_lin_vel);
    link->SetAngularVel(entity_ang_vel);
  }

  // 填充响应并设置成功标志为true
  _res->success = true;
}

GZ_REGISTER_WORLD_PLUGIN(GazeboRosState)

}  // namespace gazebo_ros
