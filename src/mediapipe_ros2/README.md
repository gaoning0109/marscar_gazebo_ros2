# Mediapipe ROS2 Wrapper
## Installation
```
sudo apt install ros-foxy-v4l2-camera -y
pip3 install mediapipe tensorflow sklearn loguru
```
## Launch
```
ros2 launch mediapipe_ros2 hand_gesture.launch.py
```
## See output in ROS2 Topic using
```
ros2 topic echo /recognized_gesture
```
