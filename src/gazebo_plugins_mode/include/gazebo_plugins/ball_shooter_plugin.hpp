/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef VRX_GAZEBO_BALL_SHOOTER_PLUGIN_HH_
#define VRX_GAZEBO_BALL_SHOOTER_PLUGIN_HH_
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <memory>
#include <mutex>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

namespace gazebo
{
/**
 * \brief 该插件用于模拟一个球发射器，当接收到ROS消息时，会发射一个炮弹。发射过的炮弹会被复用（通过瞬间移动到上一次发射前的位置）。
 *
 * 此插件接受以下SDF参数：
 * * 必需参数：
 *   <projectile>：作为炮弹使用的对象需要在<projectile>元素中声明，并包含以下属性：
 *      * <model_name>：作为炮弹使用的模型名称。这是必需的参数。
 *      * <link_name>：在炮弹模型内部，施加力以发射射击器的链接名称。这也是必需的参数。
 *      * <frame>：如果存在此参数，则<pose>参数将基于此链接/模型的坐标系。否则，默认使用世界坐标系。这是一个可选参数。
 *      * <pose>：发射前炮弹的位姿。这是可选参数，默认值为{0 0 0 0 0 0}。
 *
 * * 可选参数：
 *   <num_shots> - 允许的最大发射次数。默认值为UINT_MAX（无限制）。
 *   <shot_force> - 施加于炮弹上的力（牛顿）。默认值为250 N。
 *   <topic> - 触发发射动作的ROS主题名称。默认值为"/ball_shooter/fire"。
 *
 * 下面是一个示例配置：
 * <plugin name="ball_shooter_plugin" filename="libball_shooter_plugin.so">
 *   <projectile>
 *     <model_name>a_projectile</model_name>
 *     <link_name>link</link_name>
 *     <frame>my_robot/ball_shooter_link</frame>
 *     <pose>0.2 0 0 0 0 0</pose>
 *   </projectile>
 *   <shot_force>250</shot_force>
 *   <topic>my_robot/ball_shooter/fire</topic>
 * </plugin>
 */
// 定义一个名为BallShooterPlugin的类，该类继承自ModelPlugin基类，用于在Gazebo物理仿真环境中模拟球发射器的行为。
class BallShooterPlugin : public ModelPlugin
{
  // \brief 构造函数，默认构造。
  public: BallShooterPlugin() = default;

  // \brief 继承自ModelPlugin的加载方法，在模型加载时调用以初始化插件参数和设置。
  //       此方法接收指向当前模型（physics::ModelPtr _model）和SDF元素（sdf::ElementPtr _sdf）的指针作为参数。
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // \brief 继承自ModelPlugin的更新方法，在每个仿真循环中被调用，执行必要的更新逻辑。
  private: virtual void Update();

  // \brief 当接收到新的发射消息时调用的回调函数。此函数由订阅的主题触发。
  //       参数为ROS空消息类型的ConstPtr引用，但在此示例中未使用(_msg Unused)。
  private: void OnFire(const std_msgs::msg::Empty::ConstPtr &_msg);

  // \brief 使用互斥锁保护在回调函数中使用的成员变量，确保线程安全。
  private: std::mutex mutex;

  // \brief 创建Nodehandle以与ROS系统集成，允许插件与ROS节点进行通信。
  private: gazebo_ros::Node::SharedPtr ros_node_;

  // \brief 记录允许的最大射击次数，初始值设为无限制（UINT_MAX）。
  private: unsigned int remainingShots = UINT_MAX;

  // \brief 存储要施加到炮弹上的力（牛顿），默认为250 N。
  private: double shotForce = 250;

  // \brief 订阅一个新的主题，当发布消息时会发射新的炮弹。
  private: rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fireSub;

  // \brief 指向作为炮弹的模型对象的指针。
  private: physics::ModelPtr projectileModel;

  // \brief 指向作为炮弹的链接对象的指针。
  private: physics::LinkPtr projectileLink;

  // \brief 提供炮弹位姿参考系的链接或模型对象指针。
  public: physics::EntityPtr frame;

  // \brief 存储发射前炮弹应放置的位姿信息，初始值为原点（{0, 0, 0}位置和{0, 0, 0}角度）。
  public: ignition::math::Pose3d pose = ignition::math::Pose3d::Zero;

  // \brief 连接Gazebo事件回调到插件的Update函数的连接指针，以便在仿真循环中正确执行更新操作。
  private: event::ConnectionPtr updateConnection;

  // \brief 标记是否准备好发射炮弹，当shotReady为true时可以发射。
  private: bool shotReady = false;
};
}
#endif
