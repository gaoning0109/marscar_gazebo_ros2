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

/*
 * \brief  Simple model controller that uses a twist message to move1 an entity on the xy plane.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * \date  29 July 2013
 */

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include "gazebo_plugins/gazebo_ros_3d_move.hpp"
#include <memory>
#include <string>

namespace gazebo_plugins
{
// 定义一个名为GazeboRos3DMovePrivate的类
class GazeboRos3DMovePrivate
{
public:
  // 回调函数，在每个仿真迭代时被调用
  // \param[in] _info 更新后的仿真信息
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  // 当接收到速度命令时的回调函数
  // \param[in] _msg 扭矩命令消息
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  // 更新里程计数据
  // \param[in] _current_time 当前仿真时间
  void UpdateOdometry(const gazebo::common::Time & _current_time);

  // 发布里程计坐标变换
  // \param[in] _current_time 当前仿真时间
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  // 指向GazeboROS节点的智能指针
  gazebo_ros::Node::SharedPtr ros_node_;

  // 速度命令订阅器
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // 里程计发布器
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  // 用于广播TF（坐标变换）的智能指针
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  // 接收到的目标速度命令
  geometry_msgs::msg::Twist target_cmd_vel_;
  int seq; // 序列号

  // 存储最新的里程计消息
  nav_msgs::msg::Odometry odom_;

  // 指向世界实例的指针
  gazebo::physics::WorldPtr world_;

  // 指向模型实例的指针
  gazebo::physics::ModelPtr model_;
  gazebo::physics::EntityPtr entity_; // 可能指向模型或link等实体

  // 连接到在每次世界迭代时调用的事件
  gazebo::event::ConnectionPtr update_connection_;

  // 保护在回调中访问的变量
  std::mutex lock_;

  // 更新周期（秒）
  double update_period_;

  // 发布周期（秒）
  double publish_period_;

  // 上次更新时间
  gazebo::common::Time last_update_time_;

  // 上次发布时间
  gazebo::common::Time last_publish_time_;

  // 里程计坐标系ID
  std::string odometry_frame_;

  // 机器人基座坐标系ID
  std::string robot_base_frame_;

  // 是否发布里程计消息的标志
  bool publish_odom_;

  // 是否发布里程计到世界坐标的变换的标志
  bool publish_odom_tf_;
};

// 定义GazeboRos3DMove类的构造函数
GazeboRos3DMove::GazeboRos3DMove()
: impl_(std::make_unique<GazeboRos3DMovePrivate>())
{
  // 使用C++14中的std::make_unique创建一个私有成员变量impl_（类型为GazeboRos3DMovePrivate的智能指针）
  // 这里在构造函数初始化列表中完成对象的实例化，确保每次创建GazeboRos3DMove对象时都会有一个与之关联的GazeboRos3DMovePrivate对象实例
}

// 定义GazeboRos3DMove类的析构函数
GazeboRos3DMove::~GazeboRos3DMove()
{
  // 当GazeboRos3DMove对象被销毁时，其内部持有的std::unique_ptr<impl_>也会自动释放，从而正确清理GazeboRos3DMovePrivate对象占用的资源
}

// 定义GazeboRos3DMove类的Load方法
void GazeboRos3DMove::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // 将传入的模型指针赋值给私有成员变量impl_->model_
  impl_->model_ = _model;

  // 获取模型所在的世界对象并存储在impl_->world_中
  impl_->world_ = _model->GetWorld();

  // 初始化ROS节点
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // 获取QoS配置
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // 配置里程计参数
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first; // 设置默认里程计坐标系ID为"odom"
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first; // 设置默认机器人基座坐标系ID为"base_footprint"

  /////////////////////////////
  // 从SDF文件获取并设置pos_seq参数，作为序列号
  auto pos_seq = _sdf->Get<int>("pos_seq", 0).first;
  impl_->seq = pos_seq;
  std::cout << pos_seq << std::endl; // 输出pos_seq的值
//////////////////////////////////

  // 更新率设定
  auto update_rate = _sdf->Get<double>("update_rate", 20.0).first; // 默认更新率为20Hz
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate; // 计算实际更新周期
  } else {
    impl_->update_period_ = 0.0; // 如果更新率为0或负数，则设置为0
  }
  impl_->last_update_time_ = impl_->world_->SimTime(); // 记录上一次更新时间

  // 发布率设定
  auto publish_rate = _sdf->Get<double>("publish_rate", 20.0).first; // 默认发布率为20Hz
  if (publish_rate > 0.0) {
    impl_->publish_period_ = 1.0 / publish_rate; // 计算实际发布周期
  } else {
    impl_->publish_period_ = 0.0; // 如果发布率为0或负数，则设置为0
  }
  impl_->last_publish_time_ = impl_->world_->SimTime(); // 记录上一次发布时间

  // 创建并订阅cmd_vel话题，用于接收速度命令
  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRos3DMovePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  // 输出订阅cmd_vel的话题名称
  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub_->get_topic_name());

  // 如果需要发布里程计信息，则创建并发布odom话题
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    // 输出发布的odom话题名称
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // 是否广播TF坐标变换
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", true).first;
  if (impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    // 输出将要广播的坐标变换关系
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
      impl_->robot_base_frame_.c_str());
  }

  // 设置里程计消息中的协方差矩阵（位置和角度）
  auto covariance_x = _sdf->Get<double>("covariance_x", 0.00001).first;
  auto covariance_y = _sdf->Get<double>("covariance_y", 0.00001).first;
  auto covariance_yaw = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // 初始化里程计消息中的协方差矩阵
  impl_->odom_.pose.covariance[0] = covariance_x;
  impl_->odom_.pose.covariance[7] = covariance_y;
  impl_->odom_.pose.covariance[14] = 1000000000000.0; // 对Z轴位置、X轴旋转和Y轴旋转高斯噪声设置较大值，表示这些维度不受信任
  impl_->odom_.pose.covariance[21] = 1000000000000.0;
  impl_->odom_.pose.covariance[28] = 1000000000000.0;
  impl_->odom_.pose.covariance[35] = covariance_yaw;

  impl_->odom_.twist.covariance[0] = covariance_x;
  impl_->odom_.twist.covariance[7] = covariance_y;
  impl_->odom_.twist.covariance[14] = 1000000000000.0;
  impl_->odom_.twist.covariance[21] = 1000000000000.0;
  impl_->odom_.twist.covariance[28] = 1000000000000.0;
  impl_->odom_.twist.covariance[35] = covariance_yaw;

  // 设置里程计消息头信息
  impl_->odom_.header.frame_id = impl_->odometry_frame_;
  impl_->odom_.child_frame_id = impl_->robot_base_frame_;

  // 监听仿真世界开始更新事件，并注册OnUpdate回调函数
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRos3DMovePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}
// 定义GazeboRos3DMove类的Reset方法
void GazeboRos3DMove::Reset()
{
  // 将上次更新时间设置为当前仿真时间
  impl_->last_update_time_ = impl_->world_->SimTime();

  // 将目标速度命令（线性与角速度）全部重置为0
  impl_->target_cmd_vel_.linear.x = 0; // 线性x轴速度归零
  impl_->target_cmd_vel_.linear.y = 0; // 线性y轴速度归零
  impl_->target_cmd_vel_.linear.z = 0; // 线性z轴速度归零
  impl_->target_cmd_vel_.angular.x = 0; // 角速度绕x轴旋转归零
  impl_->target_cmd_vel_.angular.y = 0; // 角速度绕y轴旋转归零
  impl_->target_cmd_vel_.angular.z = 0; // 角速度绕z轴旋转归零
}

// 定义GazeboRos3DMovePrivate类的OnUpdate回调方法
void GazeboRos3DMovePrivate::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
  // 计算自上次更新以来的时间差（以秒为单位）
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  // 使用std::lock_guard确保线程安全
  std::lock_guard<std::mutex> scoped_lock(lock_);

#ifdef IGN_PROFILER_ENABLE
  // 如果启用了IGN_PROFILER，开始记录"Fill ROS message"阶段的时间消耗
  IGN_PROFILE("GazeboRos3DMovePrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif

  // 当时间差大于设定的更新周期时，执行以下操作
  if (seconds_since_last_update >= update_period_)
  {
    // 获取当前模型在世界坐标系中的位姿
    ignition::math::Pose3d pose = model_->WorldPose();

    // 获取模型当前绕Z轴的yaw角（偏航角），并转换速度命令至世界坐标系下
    auto yaw = static_cast<float>(pose.Rot().Yaw());
    model_->SetLinearVel(
      ignition::math::Vector3d(
        target_cmd_vel_.linear.x * cosf(yaw) - target_cmd_vel_.linear.y * sinf(yaw),
        target_cmd_vel_.linear.y * cosf(yaw) + target_cmd_vel_.linear.x * sinf(yaw),
        target_cmd_vel_.linear.z)); // 注意：此处对z轴速度没有进行旋转转换

    // 设置模型的角速度
    model_->SetAngularVel(ignition::math::Vector3d(target_cmd_vel_.angular.x, target_cmd_vel_.angular.y, target_cmd_vel_.angular.z));

    // 更新最后更新时间戳
    last_update_time_ = _info.simTime;
  }

#ifdef IGN_PROFILER_ENABLE
  // 结束记录"Fill ROS message"阶段的时间消耗
  IGN_PROFILE_END();
#endif

  // 如果需要发布里程计信息或TF坐标变换，则检查是否达到发布周期
  if (publish_odom_ || publish_odom_tf_)
  {
    double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();

    // 若未达到发布周期则直接返回
    if (seconds_since_last_publish < publish_period_)
    {
      return;
    }

#ifdef IGN_PROFILER_ENABLE
    // 开始记录"UpdateOdometry"阶段的时间消耗
    IGN_PROFILE_BEGIN("UpdateOdometry");
#endif
    // 更新里程计数据
    UpdateOdometry(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    // 结束记录"UpdateOdometry"阶段的时间消耗
    IGN_PROFILE_END();
#endif

    // 如果需要发布里程计消息
    if (publish_odom_)
    {
#ifdef IGN_PROFILER_ENABLE
      // 开始记录"publish odometry"阶段的时间消耗
      IGN_PROFILE_BEGIN("publish odometry");
#endif
      // 发布里程计消息
      odometry_pub_->publish(odom_);
#ifdef IGN_PROFILER_ENABLE
      // 结束记录"publish odometry"阶段的时间消耗
      IGN_PROFILE_END();
#endif
    }

    // 如果需要发布里程计到世界坐标系的TF坐标变换
    if (publish_odom_tf_)
    {
#ifdef IGN_PROFILER_ENABLE
      // 开始记录"publish odometryTF"阶段的时间消耗
      IGN_PROFILE_BEGIN("publish odometryTF");
#endif
      // 发布TF坐标变换
      PublishOdometryTf(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
      // 结束记录"publish odometryTF"阶段的时间消耗
      IGN_PROFILE_END();
#endif
    }

    // 更新最后发布时间戳
    last_publish_time_ = _info.simTime;
  }
}

// 定义GazeboRos3DMovePrivate类的OnCmdVel回调方法
void GazeboRos3DMovePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  // 使用std::lock_guard确保线程安全
  std::lock_guard<std::mutex> scoped_lock(lock_);

  // 将接收到的速度命令消息赋值给私有变量target_cmd_vel_
  target_cmd_vel_ = *_msg;
}
// 定义GazeboRos3DMovePrivate类的UpdateOdometry方法
void GazeboRos3DMovePrivate::UpdateOdometry(const gazebo::common::Time &_current_time)
{
  // 获取模型在世界坐标系下的位姿，并转换为ROS消息格式（geometry_msgs::msg::Pose）
  auto pose = model_->WorldPose();
  odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);

  // 获取模型在里程计坐标系（odom_frame）下的角速度Z分量
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // 将线速度从世界坐标系转换到基座坐标系（child_frame_id，例如base_footprint）
  auto linear = model_->WorldLinearVel();
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  
  // 计算并设置线速度X、Y和Z分量（考虑了yaw角旋转）
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  odom_.twist.twist.linear.z = cosf(yaw) * linear.Z(); // 注意：此处对z轴速度没有进行旋转转换

  // 设置里程计消息的时间戳
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
}

// 定义GazeboRos3DMovePrivate类的PublishOdometryTf方法
void GazeboRos3DMovePrivate::PublishOdometryTf(const gazebo::common::Time &_current_time)
{
  // 创建一个geometry_msgs::TransformStamped消息，用于存储坐标变换信息
  geometry_msgs::msg::TransformStamped msg;

  // 设置消息头的时间戳
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // 设置消息头中的父坐标系ID（通常是odom_frame）
  msg.header.frame_id = odometry_frame_;

  // 设置消息头中的子坐标系ID（例如base_footprint）
  msg.child_frame_id = robot_base_frame_;

  // 将当前里程计的位姿转换为ROS坐标变换格式，并赋值给消息体
  msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_.pose.pose);

  // 使用TF2广播器发送这个坐标变换消息
  transform_broadcaster_->sendTransform(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRos3DMove)
}  // namespace gazebo_plugins
