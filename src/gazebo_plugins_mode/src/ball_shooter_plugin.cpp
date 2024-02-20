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

#include <string>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include "gazebo_plugins/ball_shooter_plugin.hpp"

using namespace gazebo;

//////////////////////////////////////////////////
// 加载BallShooterPlugin插件，并根据SDF参数初始化其属性
void BallShooterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // 获取当前Gazebo世界的指针
  gazebo::physics::WorldPtr world = gazebo::physics::get_world();

  // 断言检查，确保_model非空
  GZ_ASSERT(_model != nullptr, "Received NULL model pointer");

  // 解析SDF中的必选<projectile>元素
  if (!_sdf->HasElement("projectile"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile> element" << std::endl;
    return;
  }

  // 获取<projectile>元素的指针
  sdf::ElementPtr projectileElem = _sdf->GetElement("projectile");

  // 解析<projectile>下的必选<model_name>元素，该模型将作为炮弹使用
  if (!projectileElem->HasElement("model_name"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile><model_name> element\n";
    return;
  }
  
  // 获取并存储炮弹模型名称
  std::string projectileName = projectileElem->GetElement("model_name")->Get<std::string>();

  // 根据Gazebo版本获取炮弹模型实体
#if GAZEBO_MAJOR_VERSION >= 8
  this->projectileModel = world->ModelByName(projectileName);
#else
  this->projectileModel = world->GetModel(projectileName);
#endif

  // 检查炮弹模型是否存在
  if (!this->projectileModel)
  {
    gzerr << "BallShooterPlugin: The model '" << projectileName
          << "' does not exist" << std::endl;
    return;
  }

  // 解析<projectile>下的必选<link_name>元素
  if (!projectileElem->HasElement("link_name"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile><link_name> element\n";
    return;
  }

  // 获取并存储炮弹链接名称
  std::string projectileLinkName =
    projectileElem->GetElement("link_name")->Get<std::string>();

  // 获取炮弹链接实体
  this->projectileLink = this->projectileModel->GetLink(projectileLinkName);

  // 检查炮弹链接是否存在
  if (!this->projectileLink)
  {
    gzerr << "BallShooterPlugin: The link '" << projectileLinkName
          << "' does not exist within '" << projectileName << "'" << std::endl;
    return;
  }

  // 解析可选<frame>元素（如果存在）
  std::string frameName;
  if (projectileElem->HasElement("frame"))
  {
    frameName = projectileElem->Get<std::string>("frame");
    
    // 根据Gazebo版本获取框架实体
#if GAZEBO_MAJOR_VERSION >= 8
    this->frame = world->EntityByName(frameName);
#else
    this->frame = world->GetEntity(frameName);
#endif

    // 检查框架实体是否存在及类型是否为链接或模型
    if (!this->frame || (!this->frame->HasType(physics::Base::LINK) &&
                         !this->frame->HasType(physics::Base::MODEL)))
    {
      this->frame = nullptr;
      frameName = "";
      gzerr << "<frame> tag must list the name of a link or model" << std::endl;
    }
  }

  // 解析可选<pose>元素（如果存在），并存储炮弹发射起始位姿
  if (projectileElem->HasElement("pose"))
    this->pose = projectileElem->Get<ignition::math::Pose3d>("pose");

  // 初始化默认发射主题名，并从SDF中读取（如果提供）
  std::string topic = "/ball_shooter/fire";
  if (_sdf->HasElement("topic"))
    topic = _sdf->GetElement("topic")->Get<std::string>();

  // 从SDF中读取可选<num_shots>元素（如果提供）
  if (_sdf->HasElement("num_shots"))
    this->remainingShots = _sdf->GetElement("num_shots")->Get<unsigned int>();

  // 从SDF中读取可选<shot_force>元素（如果提供）
  if (_sdf->HasElement("shot_force"))
    this->shotForce = _sdf->GetElement("shot_force")->Get<double>();

  // 初始化ROS节点引用
  // this->rosNodeHandle.reset(new ros::NodeHandle());
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // 创建一个订阅者，监听指定主题以触发发射事件
  // 在Gazebo ROS2版本中使用回调和create_subscription方式
  this->fireSub  = this->ros_node_->create_subscription<std_msgs::msg::Empty>(
    topic,1,std::bind(&BallShooterPlugin::OnFire, this, std::placeholders::_1));

  // 将更新函数连接到世界更新开始事件
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&BallShooterPlugin::Update, this));
}
int i =0;
// BallShooterPlugin 的更新函数
void BallShooterPlugin::Update()
{
  // 使用互斥锁保证线程安全
  std::lock_guard<std::mutex> lock(this->mutex);

  // 如果当前未准备好射击，或者炮弹模型或链接为空，则直接返回
  if (!shotReady || !this->projectileModel || !this->projectileLink)
    return;

  // 根据帧信息设置新的炮弹位姿
  ignition::math::Pose3d projectilePose = this->pose;
  if (this->frame)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    auto framePose = this->frame->WorldPose();
#else
    auto framePose = this->frame->GetWorldPose().Ign();
#endif
    // 计算世界坐标系下炮弹的最终位姿
    ignition::math::Matrix4d transMat(framePose);
    ignition::math::Matrix4d poseLocal(this->pose);
    projectilePose = (transMat * poseLocal).Pose();
  }

  // 获取Gazebo世界指针
  gazebo::physics::WorldPtr world1 = gazebo::physics::get_world();

  // 遍历所有模型
  for (auto model : world1->Models())
  {
    // 获取当前模型名称
    std::string model_name = model->GetName();

    // 检查模型名称是否符合"blue_projectile" + 当前计数器 i 的格式
    if (("blue_projectile" + std::to_string(i)) == model_name)
    {
      // 设置模型的世界位姿为计算出的炮弹位姿，并清零线性和角速度
      model->SetWorldPose(projectilePose);
      model->SetLinearVel(ignition::math::Vector3d::Zero);
      model->SetAngularVel(ignition::math::Vector3d::Zero);

      // 获取模型中的链接（假设名为 "link"）
      gazebo::physics::LinkPtr jectileLink = model->GetLink("link");

      // 对链接施加力（在此例中为沿X轴的 shotForce 大小的力）
      jectileLink->AddLinkForce({this->shotForce, 0, 0});
    }
  }

  // 更新并循环计数器 i，当达到30时重置为0
  i += 1;
  if (i >= 30)
    i = 0;

  // 减少剩余射击次数
  --this->remainingShots;

  // 射击完成后，设置 shotReady 为 false 表示准备就绪状态结束
  shotReady = false;
}
// BallShooterPlugin 的 OnFire 回调函数，当接收到发射信号时被调用
void BallShooterPlugin::OnFire(const std_msgs::msg::Empty::ConstPtr &_msg)
{
  // 使用互斥锁保证线程安全
  std::lock_guard<std::mutex> lock(this->mutex);

  // 检查剩余射击次数是否已达到最大值
  if (this->remainingShots <= 0)
  {
    // 如果剩余次数为0或负数，输出提示信息并忽略此次发射请求
    gzdbg << "BallShooterPlugin: Maximum number of shots already reached. "
          << "Request ignored" << std::endl;
    return;
  }

  // 设置 shotReady 标志为 true，表示炮弹已准备好进行下一次射击
  this->shotReady = true;
}

// 注册 BallShooterPlugin 为模型插件
GZ_REGISTER_MODEL_PLUGIN(BallShooterPlugin);