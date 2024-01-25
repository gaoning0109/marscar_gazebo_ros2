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
void BallShooterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gazebo::physics::WorldPtr world = gazebo::physics::get_world();

  GZ_ASSERT(_model != nullptr, "Received NULL model pointer");

  // Parse the required <projectile> element.
  if (!_sdf->HasElement("projectile"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile> element" << std::endl;
    return;
  }

  sdf::ElementPtr projectileElem = _sdf->GetElement("projectile");

  // Parse the required <projectile><model_name> used as projectile.
  if (!projectileElem->HasElement("model_name"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile><model_name> element\n";
    return;
  }

  std::string projectileName = projectileElem->GetElement("model_name")->Get<std::string>();

#if GAZEBO_MAJOR_VERSION >= 8
  this->projectileModel = world->ModelByName(projectileName);

#else
  this->projectileModel = world->GetModel(projectileName);
#endif
  if (!this->projectileModel)
  {
    gzerr << "BallShooterPlugin: The model '" << projectileName
          << "' does not exist" << std::endl;
    return;
  }

  // Parse the required <projectile><link_name>
  if (!projectileElem->HasElement("link_name"))
  {
    gzerr << "BallShooterPlugin: Missing <projectile><link_name> element\n";
    return;
  }

  std::string projectileLinkName =
    projectileElem->GetElement("link_name")->Get<std::string>();

  this->projectileLink = this->projectileModel->GetLink(projectileLinkName);
  if (!this->projectileLink)
  {
    gzerr << "BallShooterPlugin: The link '" << projectileLinkName
          << "' does not exist within '" << projectileName << "'" << std::endl;
    return;
  }

  // Parse <frame> if available.
  std::string frameName;
  if (projectileElem->HasElement("frame"))
  {
    frameName = projectileElem->Get<std::string>("frame");
#if GAZEBO_MAJOR_VERSION >= 8
    this->frame = world->EntityByName(frameName);
#else
    this->frame = world->GetEntity(frameName);
#endif
    if (!this->frame)
    {
      gzerr << "The frame '" << frameName << "' does not exist" << std::endl;
      frameName = "";
    }
    else if (!this->frame->HasType(physics::Base::LINK) &&
             !this->frame->HasType(physics::Base::MODEL))
    {
      this->frame = nullptr;
      frameName = "";
      gzerr << "<frame> tag must list the name of a link or model" << std::endl;
    }
  }

  // Parse <pose> if available.
  if (projectileElem->HasElement("pose"))
    this->pose = projectileElem->Get<ignition::math::Pose3d>("pose");

  // Parse <topic> if available.
  std::string topic = "/ball_shooter/fire";
  if (_sdf->HasElement("topic"))
    topic = _sdf->GetElement("topic")->Get<std::string>();

  // Parse <num_shots> if available.
  if (_sdf->HasElement("num_shots"))
    this->remainingShots = _sdf->GetElement("num_shots")->Get<unsigned int>();

  // Parse <shot_force> if available.
  if (_sdf->HasElement("shot_force"))
    this->shotForce = _sdf->GetElement("shot_force")->Get<double>();

  // Initialise the ros handle.
  // this->rosNodeHandle.reset(new ros::NodeHandle());
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);
  // this->fireSub = this->rosNodeHandle->subscribe(
  //   topic, 1, &BallShooterPlugin::OnFire, this);
    this->fireSub  = this->ros_node_->create_subscription<std_msgs::msg::Empty>(
    topic,1,std::bind(&BallShooterPlugin::OnFire, this, std::placeholders::_1));
  // Connect the update function to the world update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&BallShooterPlugin::Update, this));

}
 int i =0;
//////////////////////////////////////////////////
void BallShooterPlugin::Update()
{

  std::lock_guard<std::mutex> lock(this->mutex);

  if (!shotReady || !this->projectileModel || !this->projectileLink)
    return;

  // Set the new pose of the projectile based on the frame.
  ignition::math::Pose3d projectilePose = this->pose;
  if (this->frame)
  {
#if GAZEBO_MAJOR_VERSION >= 8
    auto framePose = this->frame->WorldPose();
#else
    auto framePose = this->frame->GetWorldPose().Ign();
#endif
    ignition::math::Matrix4d transMat(framePose);
    ignition::math::Matrix4d poseLocal(this->pose);
    projectilePose = (transMat * poseLocal).Pose();
  }
  gazebo::physics::WorldPtr world1 = gazebo::physics::get_world();
  gazebo::physics::LinkPtr jectileLink;

    for (auto model : world1->Models())
    {
        std::string model_name = model->GetName();
        if ("blue_projectile" + std::to_string(i) == model_name)
        {
            model->SetWorldPose(projectilePose);
            model->SetLinearVel(ignition::math::Vector3d::Zero);
            model->SetAngularVel(ignition::math::Vector3d::Zero);
            jectileLink = model->GetLink("link");
            jectileLink->AddLinkForce({this->shotForce, 0, 0});
        }
    }
  i+=1;
  if (i>=30)
    i=0;

  --this->remainingShots;
  shotReady = false;
}

//////////////////////////////////////////////////
void BallShooterPlugin::OnFire(const std_msgs::msg::Empty::ConstPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->remainingShots <= 0)
  {
    gzdbg << "BallShooterPlugin: Maximum number of shots already reached. "
          << "Request ignored" << std::endl;
    return;
  }

  this->shotReady = true;
}

GZ_REGISTER_MODEL_PLUGIN(BallShooterPlugin);
