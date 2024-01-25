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
# spawn_x_val = '-2.0'
# spawn_y_val = '-0.5'
# spawn_z_val = '0.5'
# spawn_yaw_val = '0.0'
spawn_x_val = '1.0'
spawn_y_val = '-0.0'
spawn_z_val = '0.3'
spawn_yaw_val = '0.0'
def generate_launch_description():

    marscar_gazebo_ros2 = os.path.join(
        get_package_share_directory('marscar_gazebo_ros2'))
    world_file_path = 'worlds/sand.world'
    world_path = os.path.join(marscar_gazebo_ros2, world_file_path)
    xacro_file = os.path.join(marscar_gazebo_ros2,
                              'urdf',
                              'marscar_blender.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    params = {'robot_description': doc.toxml()}
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                 launch_arguments={'world': world_path}.items())

    node_robot_state_publisher = Node(

        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[params],
        
        
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'mars_car',
                                   '-x',spawn_x_val,
                                    '-y',spawn_y_val,
                                    '-z',spawn_z_val,
                                    '-Y',spawn_yaw_val
                                     ],
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
    spawn_camera_controller = Node(
  
        package="controller_manager",
        executable="spawner",
        arguments=["mars_camera_controllers","--controller-manager", "controller_manager"],
        output="screen",
  
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    mars_cmdvel = Node(
        name = 'mars_cmdvel',
        package='marscar_gazebo_ros2',
        executable='mars_cmdvel.py',
        output = 'screen',
    )
    joy_shooter = Node(
        name = 'joy_shooter',
        package='marscar_gazebo_ros2',
        executable='joy_shooter.py',
        output = 'screen',
    )
    keep_balance = Node(
        name = 'keep_balance',
        package='marscar_gazebo_ros2',
        executable='keep_balance.py',
        output = 'screen',
    )
    # joy2twist = Node(
    #     name = 'joy2twist',
    #     package='marscar_gazebo_ros2',
    #     executable='joy2twist.py',
    #     output = 'screen',
    # )
    joy_params = os.path.join(get_package_share_directory('marscar_gazebo_ros2'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time':True}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )
    twist_mux_params = os.path.join(get_package_share_directory('marscar_gazebo_ros2'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/cmd_vel_mux_out')]
        )

    octomap_server_params = os.path.join(get_package_share_directory('marscar_gazebo_ros2'),'config','octomap_server.yaml')

    octomap_server = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[octomap_server_params],
            remappings=[('/cloud_in','/velodyne_points')]
         )
    ld = LaunchDescription()
    actions  = [

         
        node_robot_state_publisher,
        spawn_entity,
        spawn_car_controller,
        spawn_car_wheel_controller,
        spawn_car_arm_controller,
        joint_broad_spawner,
        mars_cmdvel,
        keep_balance,
        joy_node,
        teleop_node,
        twist_mux,
        spawn_camera_controller,
        joy_shooter
        # octomap_server
        
    ]



    group = GroupAction(actions)

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