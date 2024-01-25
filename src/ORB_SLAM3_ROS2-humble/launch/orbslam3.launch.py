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

from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction



from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    orb_slam3 = Node(

        package='orbslam3',
        executable='mono',
        output='both',
        arguments=['/home/hp/mars_car_ws/src/ORB_SLAM3_ROS2-humble/vocabulary/ORBvoc.txt',
        '/home/hp/mars_car_ws/src/ORB_SLAM3_ROS2-humble/config/monocular/EuRoC.yaml'],
         parameters=[{'use_sim_time': True}],
    
    )

    ld = LaunchDescription()
    actions  = [

         

        orb_slam3
        
    ]



    group = GroupAction(actions)

    ld.add_action(group)


    # ld.add_action(joint_state_broadcaster_spawner)
    return LaunchDescription([
        ld,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

    ])