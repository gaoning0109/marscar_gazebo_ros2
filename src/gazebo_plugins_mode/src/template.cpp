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

#include <gazebo/physics/Model.hh>
#include "gazebo_plugins/template.hpp"
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

// 定义命名空间
namespace gazebo_plugins
{
    // 创建一个私有类，用于封装PIMPL（Pointer to Implementation）设计模式中的私有数据成员
    class GazeboRosTemplatePrivate
    {
    public:
        // 公开成员变量：连接到世界更新事件，当此连接存活时回调函数会被调用
        gazebo::event::ConnectionPtr update_connection_;

        // 公开成员变量：用于ROS通信的Gazebo ROS节点
        gazebo_ros::Node::SharedPtr ros_node_;
    };

    // GazeboRosTemplate类构造函数，初始化PIMPL对象
    GazeboRosTemplate::GazeboRosTemplate()
        : impl_(std::make_unique<GazeboRosTemplatePrivate>())
    {
    }

    // GazeboRosTemplate类析构函数
    GazeboRosTemplate::~GazeboRosTemplate()
    {
    }

    // 加载方法，在Gazebo模型加载时调用
    void GazeboRosTemplate::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        // 使用传入的SDF参数创建GazeboRos节点，以处理如命名空间和重映射等通用选项
        impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

        // 输出当前模型名称的日志信息
        RCLCPP_INFO(impl_->ros_node_->get_logger(), model->GetName().c_str());

        // 建立连接，使OnUpdate函数在每个模拟迭代时被调用
        // 若不需要每帧更新，则移除此连接和回调函数
        impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&GazeboRosTemplate::OnUpdate, this));
    }

    // 更新回调函数，在每次仿真迭代时执行
    void GazeboRosTemplate::OnUpdate()
    {
        // 在此处实现每帧需要执行的操作
    }

    // 注册此插件到模拟器中
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosTemplate)
}  // namespace gazebo_plugins