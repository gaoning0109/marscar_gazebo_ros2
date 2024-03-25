cmake_minimum_required(VERSION 3.5.0)
project(marscar_gazebo_ros2)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)

include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
install(DIRECTORY
  launch
  config
  urdf
  models
  worlds
  scripts
  rviz2
  msgs
  meshes
  sensor_description
  DESTINATION share/${PROJECT_NAME}/
)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msgs/msg/Cableposseqq.msg"
  "msgs/srv/BallShooter.srv"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS 
        scripts/mars_cmdvel.py
        scripts/joy2twist.py
        scripts/keep_balance.py      
        scripts/joy_shooter.py      
        scripts/joy_cart.py 
        scripts/mars_rover_keyboard.py
        scripts/lqr_control.py
        scripts/joy_twist_plugindrive.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
