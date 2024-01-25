import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
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

    hands = launch_ros.actions.Node(
        package="mediapipe_ros2", executable="hands",output='screen'
    )
    set_pose_entity = launch_ros.actions.Node(
        package="mediapipe_ros2", executable="set_pose_entity",output='screen'
    )

    video_device = LaunchConfiguration('video_device', default='/dev/video0')
    webcam = launch_ros.actions.Node(
        package="v4l2_camera", 
        executable="v4l2_camera_node",
        parameters=[
            {"image_size": [640,480]},
            {"video_device": video_device},],
        remappings=[
            ('/image_raw', '/camera1/image_raw')]

    )
    gesture_to_cmdvel = launch_ros.actions.Node(
        package="mediapipe_ros2", executable="gesture_to_cmdvel"
    )
    marscar_gazebo_ros2 = os.path.join(
        get_package_share_directory('marscar_gazebo_ros2'))
    world_file_path = 'worlds/hand_detect.world'
    world_path = os.path.join(marscar_gazebo_ros2, world_file_path)
    world_path = os.path.join(marscar_gazebo_ros2, world_file_path)


    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                 launch_arguments={'world': world_path}.items())

    return launch.LaunchDescription([
        hands,
        webcam,
        gesture_to_cmdvel,
        set_pose_entity,
        gazebo
    ])
