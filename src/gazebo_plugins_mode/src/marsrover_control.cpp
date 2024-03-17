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

#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo_plugins/marsrover_control.hpp"
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sdf/sdf.hh>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace gazebo_plugins
{
class GazeboRosMarsRoverDrivePrivate
{
public:

/// 表示车的关节
enum {

  /// 前右轮
  FRONT_RIGHT, // 前部右侧车轮

  /// 前左轮
  FRONT_LEFT, // 前部左侧车轮

  /// 中间左轮
  MIDDLE_LEFT, // 中部左侧车轮
  MIDDLE_RIGHT, // 中部右侧车轮

  /// 后右轮
  REAR_RIGHT, // 后部右侧车轮
  REAR_LEFT, // 后部左侧车轮
  
  /////////////////////////////////
  /// 右转向关节
  STEER_RIGHT, // 右侧转向关节

  /// 左转向关节
  STEER_LEFT, // 左侧转向关节

  STEER_REAR_RIGHT, // 后部右侧转向关节
  STEER_REAR_LEFT, // 后部左侧转向关节

  /// 方向盘关节
  STEER_WHEEL // 主动方向盘或全向转向控制关节

};

  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  void OnCmdVel(geometry_msgs::msg::Twist::SharedPtr _msg);

  /**
   * \brief 提取圆柱体或球体碰撞形状的半径
   *
   * \param[in] _coll 指向碰撞形状结构体的指针
   *
   * \return 如果碰撞形状有效，则返回该形状的半径值
   * \return 如果碰撞形状无效，则返回0
   */
// 获取碰撞体半径的方法
double CollisionRadius(const gazebo::physics::CollisionPtr &_coll);

// 根据世界状态更新里程计信息
void UpdateOdometryWorld();

// 发布里程计变换信息
/// \param[in] _current_time 当前仿真时间
void PublishOdometryTf(const gazebo::common::Time &_current_time);

// 发布车轮变换信息
/// \param[in] _current_time 当前仿真时间
void PublishWheelsTf(const gazebo::common::Time &_current_time);

// 发布里程计消息至消息队列
/// \param[in] _current_time 当前仿真时间
void PublishOdometryMsg(const gazebo::common::Time &_current_time);

// ROS节点指针
gazebo_ros::Node::SharedPtr ros_node_;

// 速度命令订阅器，用于接收速度指令
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

// 里程计发布器，发布里程计消息
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

// 距离发布器，发布距离信息
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;

// 转向角发布器，发布转向角信息
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steerangle_pub_;

// 事件连接指针，用于在每一步仿真迭代时触发特定方法
gazebo::event::ConnectionPtr update_connection_;

// 轮子关节指针集合
std::vector<gazebo::physics::JointPtr> joints_;

// 模型指针，指向当前模型实体
gazebo::physics::ModelPtr model_;
// 注释：车轮之间的距离，以米为单位。
double wheel_separation_;

// 注释：前轴与后轴之间的距离（轴距），以米为单位。
double wheel_base_;

// 注释：后轮半径大小，以米为单位。
double wheel_radius_;

// 注释：转向盘与前轮之间的转角比值，即方向盘转过的角度与前轮实际转过的角度之间的比率。
double steering_ratio_ = 0;

// 注释：最大转向角，此处可能是笔误，应当改为“Max steering angle of the steering system”。
double max_speed_ = 0;

// 注释：轮胎的最大转向角度，单位为弧度。
double max_steer_ = 0;

// 注释：用于广播（Publish）坐标变换（TF）的类对象，这是一个智能指针，指向`tf2_ros::TransformBroadcaster`对象。
std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

// 注释：用于保护在回调函数中访问的共享变量的互斥锁，防止并发访问导致的数据不一致问题。
std::mutex lock_;

// 注释：接收到的X轴方向上的目标线速度，单位为米/秒（m/s）。
double target_linear_{0.0};

// 注释：接收到的Z轴方向上的目标角速度，单位为弧度/秒（rad/s）。
double target_rot_{0.0};

// 注释：更新周期，单位为秒（s）。
double update_period_;

// 注释：上次更新的时间戳，使用Gazebo的`common::Time`类型表示。
gazebo::common::Time last_update_time_;

// 注释：里程计坐标系的标识符（ID）字符串。
std::string odometry_frame_;
// 保留最新的里程计消息
nav_msgs::msg::Odometry odom_;

// 保留最新的距离信息消息
std_msgs::msg::Float32 distance_;

// 保留最新的转向角信息消息
std_msgs::msg::Float32 steerangle_;

// 机器人底座坐标系ID
std::string robot_base_frame_;

// 是否发布里程计消息的标志位
bool publish_odom_;

// 是否发布行驶距离信息的标志位
bool publish_distance_;

// 是否发布转向角信息的标志位
bool publish_steerangle_;

// 是否发布车轮相对于底座坐标系的变换信息
bool publish_wheel_tf_;

// 是否发布里程计坐标系相对于世界坐标系的变换信息
bool publish_odom_tf_;

// 里程计中的协方差数组（3x3矩阵存储为一维数组）
double covariance_[3];

// 左转向PID控制器
gazebo::common::PID pid_left_steering_;
gazebo::common::PID pid_rear_left_steering_;

// 右转向PID控制器
gazebo::common::PID pid_right_steering_;
gazebo::common::PID pid_rear_right_steering_;

// 线性速度PID控制器
gazebo::common::PID pid_linear_vel_;
};

GazeboRosMarsRoverDrive::GazeboRosMarsRoverDrive()
: impl_(std::make_unique<GazeboRosMarsRoverDrivePrivate>())
{
}

GazeboRosMarsRoverDrive::~GazeboRosMarsRoverDrive()
{
}

void GazeboRosMarsRoverDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  auto world = impl_->model_->GetWorld();
  auto physicsEngine = world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  impl_->joints_.resize(13);

  auto steering_wheel_joint =
    _sdf->Get<std::string>("steering_wheel_joint", "steering_wheel_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_WHEEL] =
    _model->GetJoint(steering_wheel_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_WHEEL]) {
    RCLCPP_WARN(
      impl_->ros_node_->get_logger(),
      "Steering wheel joint [%s] not found.", steering_wheel_joint.c_str());
    impl_->joints_.resize(12);
  }

  auto front_right_joint = _sdf->Get<std::string>("front_right_joint", "front_right_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::FRONT_RIGHT] = _model->GetJoint(front_right_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::FRONT_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Front right wheel joint [%s] not found, plugin will not work.", front_right_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto front_left_joint = _sdf->Get<std::string>("front_left_joint", "front_left_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::FRONT_LEFT] = _model->GetJoint(front_left_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::FRONT_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Front left wheel joint [%s] not found, plugin will not work.", front_left_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto rear_right_joint = _sdf->Get<std::string>("rear_right_joint", "rear_right_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::REAR_RIGHT] = _model->GetJoint(rear_right_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::REAR_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Rear right wheel joint [%s] not found, plugin will not work.", rear_right_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto middle_right_joint = _sdf->Get<std::string>("middle_right_joint", "middle_right_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::MIDDLE_RIGHT] = _model->GetJoint(middle_right_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::MIDDLE_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Middle right wheel joint [%s] not found, plugin will not work.", middle_right_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto rear_left_joint = _sdf->Get<std::string>("rear_left_joint", "rear_left_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::REAR_LEFT] = _model->GetJoint(rear_left_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::REAR_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Rear left wheel joint [%s] not found, plugin will not work.", rear_left_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto middle_left_joint = _sdf->Get<std::string>("middle_left_joint", "middle_left_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::MIDDLE_LEFT] = _model->GetJoint(middle_left_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::MIDDLE_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Middle left wheel joint [%s] not found, plugin will not work.", middle_left_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto right_steering_joint =
    _sdf->Get<std::string>("right_steering_joint", "right_steering_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_RIGHT] =
    _model->GetJoint(right_steering_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Right wheel steering joint [%s] not found, plugin will not work.",
      right_steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto right_rear_steering_joint =
    _sdf->Get<std::string>("right_rear_steering_joint", "right_rear_steering_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_REAR_RIGHT] =
    _model->GetJoint(right_rear_steering_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_REAR_RIGHT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Right rear wheel steering joint [%s] not found, plugin will not work.",
      right_rear_steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }

  auto left_steering_joint =
    _sdf->Get<std::string>("left_steering_joint", "left_steering_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_LEFT] =
    _model->GetJoint(left_steering_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Left wheel steering joint [%s] not found, plugin will not work.",
      left_steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }
  auto left_rear_steering_joint =
    _sdf->Get<std::string>("left_rear_steering_joint", "left_rear_steering_joint").first;
  impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_REAR_LEFT] =
    _model->GetJoint(left_rear_steering_joint);
  if (!impl_->joints_[GazeboRosMarsRoverDrivePrivate::STEER_REAR_LEFT]) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "Left rear wheel steering joint [%s] not found, plugin will not work.",
      left_rear_steering_joint.c_str());
    impl_->ros_node_.reset();
    return;
  }
  impl_->max_speed_ = _sdf->Get<double>("max_speed", 20.0).first;
  impl_->max_steer_ = _sdf->Get<double>("max_steer", 0.6).first;

  // Max the steering wheel can rotate
  auto max_steering_angle = _sdf->Get<double>("max_steering_angle", 7.85).first;

  // Compute the angle ratio between the steering wheel and the tires
  impl_->steering_ratio_ = impl_->max_steer_ / max_steering_angle;

  auto pid = _sdf->Get<ignition::math::Vector3d>(
    "right_steering_pid_gain", ignition::math::Vector3d::Zero).first;
  auto i_range = _sdf->Get<ignition::math::Vector2d>(
    "right_steering_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_right_steering_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  pid = _sdf->Get<ignition::math::Vector3d>(
    "left_steering_pid_gain", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>(
    "left_steering_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_left_steering_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  pid = _sdf->Get<ignition::math::Vector3d>(
    "left_rear_steering_pid_gain", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>(
    "left_rear_steering_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_rear_left_steering_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  pid = _sdf->Get<ignition::math::Vector3d>(
    "right_rear_steering_pid_gain", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>(
    "right_rear_steering_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_rear_right_steering_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());

  pid = _sdf->Get<ignition::math::Vector3d>(
    "linear_velocity_pid_gain", ignition::math::Vector3d::Zero).first;
  i_range = _sdf->Get<ignition::math::Vector2d>(
    "linear_velocity_i_range", ignition::math::Vector2d::Zero).first;
  impl_->pid_linear_vel_.Init(pid.X(), pid.Y(), pid.Z(), i_range.Y(), i_range.X());


  unsigned int id = 0;
  impl_->wheel_radius_ = impl_->CollisionRadius(
    impl_->joints_[GazeboRosMarsRoverDrivePrivate::REAR_RIGHT]->GetChild()->GetCollision(id));


  auto front_right_center_pos = impl_->joints_[GazeboRosMarsRoverDrivePrivate::FRONT_RIGHT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto front_left_center_pos = impl_->joints_[GazeboRosMarsRoverDrivePrivate::FRONT_LEFT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto rear_right_center_pos = impl_->joints_[GazeboRosMarsRoverDrivePrivate::REAR_RIGHT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();
  auto rear_left_center_pos = impl_->joints_[GazeboRosMarsRoverDrivePrivate::REAR_LEFT]->
    GetChild()->GetCollision(id)->WorldPose().Pos();

  auto distance = front_left_center_pos - front_right_center_pos;
  impl_->wheel_separation_ = distance.Length();

// 计算车轴中心间距以求得轴距（wheelbase）
// 首先，计算前后轴中心点的平均位置
// 前轴中心点位置是两个前轮中心点位置的平均值
auto front_axle_pos = (front_left_center_pos + front_right_center_pos) / 2;

// 后轴中心点位置是两个后轮中心点位置的平均值
auto rear_axle_pos = (rear_left_center_pos + rear_right_center_pos) / 2;

// 然后，轴距就是前后轴中心点之间的直线距离
distance = front_axle_pos - rear_axle_pos;

// 将计算得到的直线距离赋值给类实例的成员变量 wheel_base_
impl_->wheel_base_ = distance.Length();
  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 100.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = _model->GetWorld()->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosMarsRoverDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", false).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Advertise distance travelled
  impl_->publish_distance_ = _sdf->Get<bool>("publish_distance", false).first;
  if (impl_->publish_distance_) {
    impl_->distance_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
      "distance", qos.get_publisher_qos("distance", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise distance on [%s]",
      impl_->distance_pub_->get_topic_name());
  }

  // Advertise steering angle
  impl_->publish_steerangle_ = _sdf->Get<bool>("publish_steerangle", false).first;
  if (impl_->publish_steerangle_) {
    impl_->steerangle_pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Float32>(
      "steerangle", qos.get_publisher_qos("steerangle", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise steerangle on [%s]",
      impl_->steerangle_pub_->get_topic_name());
  }

  // Create TF broadcaster if needed
  impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", false).first;
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", false).first;
  if (impl_->publish_wheel_tf_ || impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    if (impl_->publish_odom_tf_) {
      RCLCPP_INFO(
        impl_->ros_node_->get_logger(),
        "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
        impl_->robot_base_frame_.c_str());
    }

    if (impl_->publish_wheel_tf_) {
      for (auto & joint : impl_->joints_) {
        RCLCPP_INFO(
          impl_->ros_node_->get_logger(),
          "Publishing wheel transforms between [%s], [%s] and [%s]",
          impl_->robot_base_frame_.c_str(), joint->GetName().c_str(), joint->GetName().c_str());
      }
    }
  }

  auto pose = impl_->model_->WorldPose();
  impl_->odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  impl_->odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(
    pose.Rot());

  impl_->covariance_[0] = _sdf->Get<double>("covariance_x", 0.00001).first;
  impl_->covariance_[1] = _sdf->Get<double>("covariance_y", 0.00001).first;
  impl_->covariance_[2] = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosMarsRoverDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosMarsRoverDrive::Reset()
{
  impl_->last_update_time_ = impl_->model_->GetWorld()->SimTime();

  impl_->target_linear_ = 0;
  impl_->target_rot_ = 0;
  impl_->distance_.data = 0;
  impl_->steerangle_.data = 0;
}

void GazeboRosMarsRoverDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosMarsRoverDrivePrivate::OnUpdate");
  #endif
  std::lock_guard<std::mutex> lock(lock_);

  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("UpdateOdometryWorld");
#endif
  // Update odom
  UpdateOdometryWorld();
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  if (seconds_since_last_update < update_period_) {
    return;
  }

  if (publish_distance_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish distance");
#endif
    distance_pub_->publish(distance_);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  if (publish_odom_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishOdometryMsg");
#endif
    PublishOdometryMsg(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  if (publish_wheel_tf_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishWheelsTf");
#endif
    PublishWheelsTf(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

  if (publish_odom_tf_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("PublishOdometryTf");
#endif
    PublishOdometryTf(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }

#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_BEGIN("update");
#endif
  // Current speed assuming equal for left rear and right rear
// 获取后右轮关节的速度，单位为线速度（m/s）
auto linear_vel = joints_[REAR_RIGHT]->GetVelocity(0);

// 对目标线速度进行钳位处理，限制在最大速度和最小速度之间
auto target_linear = ignition::math::clamp(target_linear_, -max_speed_, max_speed_);

// 计算当前线速度与目标线速度经过单位时间（秒）调整后的差值，并除以车轮半径
double linear_diff = linear_vel - target_linear / wheel_radius_;

// 使用线速度PID控制器更新控制量
double linear_cmd_L = pid_linear_vel_.Update(linear_diff, seconds_since_last_update);
double linear_cmd_R = linear_cmd_L;
// 确定转向角的目标值，根据target_linear_的正负号决定转向方向
// auto target_rot = target_rot_ * copysign(1.0, target_linear_);
auto target_rot = target_rot_ ;
// 对目标转向角进行钳位处理，限制在最大转向角和最小转向角之间
target_rot = ignition::math::clamp(target_rot, -max_steer_, max_steer_);

// 根据目标转向角计算正切值
double tanSteer = tan(target_rot);

// 计算左轮和右轮期望的转向角度（atan2函数用于计算坐标轴旋转角度）
auto target_left_steering =
    atan2(tanSteer, 1.0 - wheel_separation_ / 2.0 / wheel_base_ * tanSteer);
auto target_right_steering =
    atan2(tanSteer, 1.0 + wheel_separation_ / 2.0 / wheel_base_ * tanSteer);
auto target_left_rear_steering =
    atan2(-tanSteer, 1.0 + wheel_separation_ / 2.0 / wheel_base_ * (-tanSteer));
auto target_right_rear_steering =
    atan2(-tanSteer,  1.0 - wheel_separation_ / 2.0 / wheel_base_ * (-tanSteer));

if (target_linear == 0 && target_rot != 0)
{
    // 根据目标角速度的符号翻转相应一侧的转向角
    target_left_steering *= (target_rot > 0) ? -1 : 1;
    target_left_rear_steering *= (target_rot > 0) ? -1 : 1;
    target_right_steering *= (target_rot < 0) ? -1 : 1;
    target_right_rear_steering *= (target_rot < 0) ? -1 : 1;

    // 不论目标角速度正负，左右两侧的线性速度指令始终取目标角速度的相反数乘以100
    linear_cmd_L = -target_rot * 100;
    linear_cmd_R = target_rot * 100;
}



// 获取当前左轮和右轮的实际转向角度
auto left_steering_angle = joints_[STEER_LEFT]->Position(0);
auto right_steering_angle = joints_[STEER_RIGHT]->Position(0);
auto left_rear_steering_angle = joints_[STEER_REAR_LEFT]->Position(0);
auto right_rear_steering_angle = joints_[STEER_REAR_RIGHT]->Position(0);

// 计算左轮和右轮实际转向角度与目标转向角度之间的差值
double left_steering_diff = left_steering_angle - target_left_steering;
double right_steering_diff = right_steering_angle - target_right_steering;
double left_rear_steering_diff = left_rear_steering_angle - target_left_rear_steering;
double right_rear_steering_diff = right_rear_steering_angle - target_right_rear_steering;


// 使用左右轮的PID控制器分别更新转向控制量
double left_steering_cmd =
    pid_left_steering_.Update(left_steering_diff, seconds_since_last_update);
double right_steering_cmd =
    pid_right_steering_.Update(right_steering_diff, seconds_since_last_update);

double left_rear_steering_cmd =
    pid_rear_left_steering_.Update(left_rear_steering_diff, seconds_since_last_update);
double right_rear_steering_cmd =
    pid_rear_right_steering_.Update(right_rear_steering_diff, seconds_since_last_update);

// 计算平均转向角度，并转换为实际物理意义上的转向盘角度（考虑到转向比）
auto steer_wheel_angle = (left_steering_angle + right_steering_angle) * 0.5 / steering_ratio_;

// 更新steerangle_的数据，存储当前车辆的平均转向角度
steerangle_.data = (left_steering_angle + right_steering_angle) * 0.5;

  if (publish_steerangle_) {
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("publish steerangle");
#endif
    steerangle_pub_->publish(steerangle_);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
  }


  joints_[STEER_LEFT]->SetForce(0, left_steering_cmd);
  joints_[STEER_RIGHT]->SetForce(0, right_steering_cmd);
  joints_[STEER_REAR_LEFT]->SetForce(0, left_rear_steering_cmd);
  joints_[STEER_REAR_RIGHT]->SetForce(0, right_rear_steering_cmd);
  joints_[FRONT_LEFT]->SetForce(0, linear_cmd_L);
  joints_[FRONT_RIGHT]->SetForce(0, linear_cmd_R);
  joints_[MIDDLE_LEFT]->SetForce(0, linear_cmd_L);
  joints_[MIDDLE_RIGHT]->SetForce(0, linear_cmd_R);
  joints_[REAR_RIGHT]->SetForce(0, linear_cmd_R);
  joints_[REAR_LEFT]->SetForce(0, linear_cmd_L);


  if (joints_.size() == 13) {
    joints_[STEER_WHEEL]->SetPosition(0, steer_wheel_angle);
  }

  last_update_time_ = _info.simTime;
  #ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
  #endif
}

void GazeboRosMarsRoverDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_linear_ = _msg->linear.x;
  target_rot_ = _msg->angular.z;
}

double GazeboRosMarsRoverDrivePrivate::CollisionRadius(const gazebo::physics::CollisionPtr & _coll)
{
  if (!_coll || !(_coll->GetShape())) {
    return 0;
  }
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE)) {
    gazebo::physics::CylinderShape * cyl =
      dynamic_cast<gazebo::physics::CylinderShape *>(_coll->GetShape().get());
    return cyl->GetRadius();
  } else if (_coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE)) {
    gazebo::physics::SphereShape * sph =
      dynamic_cast<gazebo::physics::SphereShape *>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

void GazeboRosMarsRoverDrivePrivate::UpdateOdometryWorld()
{
  auto prev_x = odom_.pose.pose.position.x;
  auto prev_y = odom_.pose.pose.position.y;

  auto pose = model_->WorldPose();
  odom_.pose.pose.position = gazebo_ros::Convert<geometry_msgs::msg::Point>(pose.Pos());
  odom_.pose.pose.orientation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

  distance_.data += hypot(prev_x - odom_.pose.pose.position.x, prev_y - odom_.pose.pose.position.y);

  // Get velocity in odom frame
  auto linear = model_->WorldLinearVel();
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
}

void GazeboRosMarsRoverDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform.translation =
    gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_.pose.pose.position);
  msg.transform.rotation = odom_.pose.pose.orientation;

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosMarsRoverDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  for (const auto & joint : joints_) {
    auto pose = joint->GetChild()->WorldPose() - model_->WorldPose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = robot_base_frame_;
    msg.child_frame_id = joint->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}

void GazeboRosMarsRoverDrivePrivate::PublishOdometryMsg(const gazebo::common::Time & _current_time)
{
  // Set covariance
  odom_.pose.covariance[0] = covariance_[0];
  odom_.pose.covariance[7] = covariance_[1];
  odom_.pose.covariance[14] = 1000000000000.0;
  odom_.pose.covariance[21] = 1000000000000.0;
  odom_.pose.covariance[28] = 1000000000000.0;
  odom_.pose.covariance[35] = covariance_[2];

  odom_.twist.covariance[0] = covariance_[0];
  odom_.twist.covariance[7] = covariance_[1];
  odom_.twist.covariance[14] = 1000000000000.0;
  odom_.twist.covariance[21] = 1000000000000.0;
  odom_.twist.covariance[28] = 1000000000000.0;
  odom_.twist.covariance[35] = covariance_[2];

  // Set header
  odom_.header.frame_id = odometry_frame_;
  odom_.child_frame_id = robot_base_frame_;
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);

  // Publish
  odometry_pub_->publish(odom_);
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMarsRoverDrive)
}  // namespace gazebo_plugins