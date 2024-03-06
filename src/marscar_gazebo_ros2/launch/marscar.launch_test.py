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
spawn_x_val = '.0'
spawn_y_val = '.0'
spawn_z_val = '1.0'
spawn_yaw_val = '0.00'
def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    gazebo_ros2_mars_car = os.path.join(
        get_package_share_directory('gazebo_ros2_mars_car'))

    # xacro_file = os.path.join(gazebo_ros2_control_demos_path,
    #                           'urdf',
    #                           'repair_tool.xacro')
    xacro_file = os.path.join(gazebo_ros2_mars_car,
                              'urdf',
                              'mars_car.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(

        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
        
        
    )

    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-gazebo_namespace','/gazebo','-topic', 'robot_description',
    #                                '-entity', 'mars_car',
    #                                '-x',spawn_x_val,
    #                                 '-y',spawn_y_val,
    #                                 '-z',spawn_z_val, ],
    #                     output='screen')
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mars_car',
                                   '-x',spawn_x_val,
                                    '-y',spawn_y_val,
                                    '-z',spawn_z_val, ],
                        output='screen')
    spawn_car_controller = Node(
  
        package="controller_manager",
        executable="spawner",
        arguments=["mars_car_controllers","--controller-manager", "controller_manager"],
        output="screen",
  
    )
    spawn_car_wheel_controller = Node(
  
        package="controller_manager",
        executable="spawner",
        arguments=["mars_car_wheel_controllers","--controller-manager", "controller_manager"],
        output="screen",
    )
    spawn_car_arm_controller = Node(
  
        package="controller_manager",
        executable="spawner",
        arguments=["mars_car_arm_controllers","--controller-manager", "controller_manager"],
        output="screen",
  
    )
    
    ld = LaunchDescription()
    actions  = [
        # PushRosNamespace('repair_tool'), 
         
        node_robot_state_publisher,
        spawn_entity,
        spawn_car_controller,
        spawn_car_wheel_controller,
        spawn_car_arm_controller
        
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

    # Message to tf

    ld.add_action(gazebo)
    ld.add_action(group)

    ld.add_action(robot_control)

    # ld.add_action(joint_state_broadcaster_spawner)
    return LaunchDescription([
        ld,
    ])