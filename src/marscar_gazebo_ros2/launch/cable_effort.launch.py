# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration as Lc

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
def generate_launch_description():
    marscar_gazebo_ros2 = os.path.join(
        get_package_share_directory('marscar_gazebo_ros2'))

    world_file_path = 'worlds/hand_detect.world'
    world_path = os.path.join(marscar_gazebo_ros2, world_file_path)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={'world': world_path}.items()
             )


    xacro_file = os.path.join(marscar_gazebo_ros2,
                              'urdf',
                              'roll_effort.xacro')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(

        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
        
        
    )
    joint_state_broadcaster_spawner = Node(
        namespace='cable',
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    joy_cart = Node(
        name = 'joy_cart',
        package='marscar_gazebo_ros2',
        executable='joy_cart.py',
        output = 'screen',
    )
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'cartpole',
                                    '-x','0.0',
                                    '-y','0.0',
                                    '-z','30.0',
                                     ],
                        output='screen')

    spawn_cart_controller = Node(
  
        package="controller_manager",
        executable="spawner",
        arguments=["cart_controllers","--controller-manager", "/controller_manager"],
        output="screen",

        
    )
    
    ld = LaunchDescription()
    actions  = [

        node_robot_state_publisher,
        spawn_entity,
        

    ]


    group = GroupAction(actions)

    # 启动控制服务
    # 输入关节角度，输出gazebo控制
    

    robot_control = Node(
        name = 'joy',
        package='joy',
        executable='joy_node',
        output = 'screen',
    )

    ld.add_action(group)
    ld.add_action(spawn_cart_controller)
    ld.add_action(robot_control)
    ld.add_action(gazebo)
    ld.add_action(joy_cart)
    # ld.add_action(joint_state_broadcaster_spawner)
    return LaunchDescription([
        ld,
    ])