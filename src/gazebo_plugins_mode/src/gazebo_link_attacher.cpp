/*
# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: May, 2023.                                                                     #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) IFRA Gazebo-ROS2 Link Attacher. URL: https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Light.hh>

#include <gazebo/physics/Link.hh>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <memory>
#include <gazebo/rendering/Visual.hh>
#include "gazebo_ros/conversions/builtin_interfaces.hpp"
#include "gazebo_ros/conversions/geometry_msgs.hpp"

#include "gazebo_plugins/gazebo_link_attacher.hpp"  
#include <linkattacher_msgs/srv/attach_link.hpp>     
#include <linkattacher_msgs/srv/detach_link.hpp>      

// GLOBAL VARIABLE: 定义一个全局变量，它是一个存储关节结构体的向量
std::vector<JointSTRUCT> GV_joints; // 全局变量，用于存储一系列关节数据结构

// 另一个全局变量，用于存储单个JointSTRUCT类型的对象
JointSTRUCT GV_jointSTR; // 全局变量，用于存储单个关节的具体信息
namespace gazebo_ros
{

// GazeboLinkAttacherPrivate 类定义，这是一个辅助类，通常作为主类（GazeboLinkAttacher）的私有成员实现细节

class GazeboLinkAttacherPrivate
{
public:
  // ATTACH (ROS2 service) 方法：处理来自ROS2客户端的link附着请求
  void Attach(
    linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req, // 请求对象指针
    linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res); // 响应对象指针

  // DETACH (ROS2 service) 方法：处理来自ROS2客户端的link分离请求
  void Detach(
    linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
    linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res);

  // Gazebo世界指针，用于与Gazebo物理仿真环境交互
  gazebo::physics::WorldPtr world_;

  /// ROS节点指针，由gazebo_ros库管理，用于ROS通信
  gazebo_ros::Node::SharedPtr ros_node_;

  // ROS服务对象，分别用于提供link附着和分离的服务
  rclcpp::Service<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_service_;
  rclcpp::Service<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_link_service_;

  // getJoint 函数：根据给定的模型名（M1, M2）和链杆名（L1, L2），获取关节信息并存储在joint参数中
  bool getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint);
};

// GazeboLinkAttacher 类的构造函数
GazeboLinkAttacher::GazeboLinkAttacher()
: impl_(std::make_unique<GazeboLinkAttacherPrivate>()) // 使用C++14的std::make_unique创建并初始化一个私有成员对象（智能指针）impl_
{
  // 在这里可以进行类初始化时所需的任何其他设置或配置
}

// GazeboLinkAttacher 类的析构函数
GazeboLinkAttacher::~GazeboLinkAttacher()
{
  // 当GazeboLinkAttacher对象销毁时，由于impl_是一个std::unique_ptr，它会自动释放其指向的GazeboLinkAttacherPrivate实例，
  // 因此无需在此处手动释放资源
}
// GazeboLinkAttacher 类的 Load 方法，用于加载世界模型和初始化 ROS2 相关组件

void GazeboLinkAttacher::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  // 将Gazebo世界的指针赋值给私有成员变量
  impl_->world_ = _world;

  // 初始化ROS2节点，通过_sdf参数获取或创建一个与当前仿真场景关联的ROS2节点
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // 创建并启动两个ROS2服务服务器
  // ATTACHLINK服务：将请求转发到GazeboLinkAttacherPrivate类的Attach方法处理
  impl_->attach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::AttachLink>(
      "ATTACHLINK", 
      std::bind(&GazeboLinkAttacherPrivate::Attach, impl_.get(),
                std::placeholders::_1, std::placeholders::_2));

  // DETACHLINK服务：将请求转发到GazeboLinkAttacherPrivate类的Detach方法处理
  impl_->detach_link_service_ =
    impl_->ros_node_->create_service<linkattacher_msgs::srv::DetachLink>(
      "DETACHLINK",
      std::bind(&GazeboLinkAttacherPrivate::Detach, impl_.get(),
                std::placeholders::_1, std::placeholders::_2));
}

// 在此函数中，我们首先设置了Gazebo物理世界实例，然后设置了一个与Gazebo仿真环境关联的ROS2节点。
// 接着，我们创建了两个ROS2服务（"ATTACHLINK" 和 "DETACHLINK"），分别对应link附着和分离的功能，
//并将请求转发给私有成员类中的相关回调函数进行处理。
//通过std::bind和std::placeholders::_1、std::placeholders::_2来捕获并传递服务调用时的请求和响应对象。

void GazeboLinkAttacherPrivate::Attach(
  linkattacher_msgs::srv::AttachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::AttachLink::Response::SharedPtr _res)
{
  // 获取第一个link：首先根据请求中的模型名找到对应的Gazebo模型对象
  gazebo::physics::ModelPtr model1 = world_->ModelByName(_req->model1_name);
  if (!model1) {
    // 如果找不到该模型，则设置响应的success为false，并附上错误信息
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model1_name;
    return;
  }
  // 然后在模型中查找指定名称的link
  gazebo::physics::LinkPtr link1 = model1->GetLink(_req->link1_name);
  if (!link1) {
    // 如果找不到该link，则设置响应的success为false，并附上错误信息
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link1_name;
    return;
  }

  // 获取第二个link，过程同上
  gazebo::physics::ModelPtr model2 = world_->ModelByName(_req->model2_name);
  if (!model2) {
    _res->success = false;
    _res->message = "Failed to find model with name: " + _req->model2_name;
    return;
  }
  gazebo::physics::LinkPtr link2 = model2->GetLink(_req->link2_name);
  if (!link2) {
    _res->success = false;
    _res->message = "Failed to find link with name: " + _req->link2_name;
    return;
  }

  // 创建一个固定关节连接两个link
  std::string JointName = _req->model1_name + "_" + _req->link1_name + "_" + _req->model2_name + "_" + _req->link2_name + "_joint";

  // 计算两个link之间的相对位置
  ignition::math::Pose3d parent_pose = link1->WorldPose();
  ignition::math::Pose3d link_pose = link2->WorldPose();
  ignition::math::Pose3d diff = parent_pose - link_pose;

  // 如果两个link之间的距离小于2m，则创建固定关节
  if (diff.Pos().Length() < 2) {
    // 创建固定关节并关联到两个link之间
    gazebo::physics::JointPtr joint1 = model1->CreateJoint(JointName, "fixed", link1, link2);

    // 设置服务响应的成功标志和消息
    _res->success = true;
    _res->message = "ATTACHED: {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}.";
  }
  


}

void GazeboLinkAttacherPrivate::Detach(
  linkattacher_msgs::srv::DetachLink::Request::SharedPtr _req,
  linkattacher_msgs::srv::DetachLink::Response::SharedPtr _res)
{
  // 获取请求中指定的第一个模型
  gazebo::physics::ModelPtr model1 = world_->ModelByName(_req->model1_name);

  // 定义要删除的关节名称，基于请求中的模型和link名构建
  std::string JointName = _req->model1_name + "_" + _req->link1_name + "_" + _req->model2_name + "_" + _req->link2_name + "_joint";

  // 检查GV_joints全局变量中是否包含该关节（注：代码片段中未体现检查过程）
  
  // 直接从模型1中移除名为JointName的关节
  if (model1 && model1->GetJoint(JointName)) {
    model1->RemoveJoint(JointName);
    
    // 设置响应信息，表示成功分离了link
    _res->success = true;
    _res->message = "DETACHED: {" + _req->model1_name + " , " + _req->link1_name + "} -- {" + _req->model2_name + " , " + _req->link2_name + "}";
  } else {
    // 如果模型不存在或找不到相应的关节，则设置失败信息
    _res->success = false;
    _res->message = "Failed to find joint with name: " + JointName;
  }
}


bool GazeboLinkAttacherPrivate::getJoint(std::string M1, std::string L1, std::string M2, std::string L2, JointSTRUCT &joint)
{
  // 初始化一个临时关节结构体变量用于存储遍历过程中的信息
  JointSTRUCT j;

  // 遍历全局关节列表GV_joints
  for (std::vector<JointSTRUCT>::iterator it = GV_joints.begin(); it != GV_joints.end(); ++it) {
    // 将当前迭代器指向的关节结构体赋值给临时变量j
    j = *it;

    // 检查临时变量j中记录的模型和link名称是否与参数匹配
    if ((j.model1.compare(M1) == 0) && (j.model2.compare(M2) == 0) && (j.link1.compare(L1) == 0) && (j.link2.compare(L2) == 0)) {
      // 找到匹配项，将临时变量j的内容复制到输出参数joint中
      joint = j;
      
      // 返回true表示找到了指定的关节
      return true;
    }
  }

  // 如果遍历结束仍未找到匹配项，则返回false表示未找到指定的关节
  return false;
}

GZ_REGISTER_WORLD_PLUGIN(GazeboLinkAttacher)

}  // namespace gazebo_ros