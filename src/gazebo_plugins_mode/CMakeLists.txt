cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins_mode)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(include)

# GAZEBO LINK ATTACHER:

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)

find_package(rcl REQUIRED)
find_package(rmw REQUIRED)
find_package(tinyxml_vendor REQUIRED)
find_package(linkattacher_msgs REQUIRED)

#gazebo_ros_planar_move1
add_library(gazebo_ros_3d_move_mode SHARED
  src/gazebo_ros_3d_move_mode.cpp
)
target_include_directories(gazebo_ros_3d_move_mode PUBLIC include)
ament_target_dependencies(gazebo_ros_3d_move_mode
  "gazebo_dev"
  "gazebo_ros"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"

)
ament_export_libraries(gazebo_ros_3d_move_mode)

# gazebo_ros_state
add_library(gazebo_ros_state_mode SHARED
  src/gazebo_ros_state_mode.cpp
)
target_include_directories(gazebo_ros_state_mode PUBLIC include)
ament_target_dependencies(gazebo_ros_state_mode
  "rclcpp"
  "gazebo_dev"
  "gazebo_msgs"
  "gazebo_ros"

)
# target_link_libraries(gazebo_ros_state1
#   gazebo_ros_node
# )
ament_export_libraries(gazebo_ros_state_mode)

# gazebo_ros_vacuum_gripper
add_library(gazebo_ros_boom SHARED
  src/boom.cpp
)
target_include_directories(gazebo_ros_boom PUBLIC include)
ament_target_dependencies(gazebo_ros_boom
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "std_msgs"
  "std_srvs"
)
ament_export_libraries(gazebo_ros_boom)

# gazebo_ros_diff_drive
add_library(gazebo_ros_odom SHARED
  src/gazebo_ros_odom.cpp
)
target_include_directories(gazebo_ros_odom PUBLIC include)
ament_target_dependencies(gazebo_ros_odom
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_odom)
####################################################
add_library(ball_shooter_plugin SHARED
  src/ball_shooter_plugin.cpp
)
target_include_directories(ball_shooter_plugin PUBLIC include)
ament_target_dependencies(ball_shooter_plugin
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(ball_shooter_plugin)
####################################################
add_library(template SHARED
  src/template.cpp
)
target_include_directories(template PUBLIC include)
ament_target_dependencies(template
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "sensor_msgs"
)
ament_export_libraries(template)
add_library(gazebo_ros_force_mode SHARED
  src/gazebo_ros_force_mode.cpp
)
target_include_directories(gazebo_ros_force_mode PUBLIC include)
ament_target_dependencies(gazebo_ros_force_mode
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)
ament_export_libraries(gazebo_ros_force_mode)
add_library(gazebo_link_attacher SHARED
  src/gazebo_link_attacher.cpp
  # src/flexible_rope_plugin.cpp
)
target_link_libraries(gazebo_link_attacher ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_link_attacher
  "rclcpp"
  "gazebo_dev"
  "gazebo_msgs"
  "gazebo_ros"
  "linkattacher_msgs"
)
ament_export_libraries(gazebo_link_attacher)

add_library(gazebo_ros_test SHARED
  src/gazebo_ros_test.cpp
)
target_include_directories(gazebo_ros_test PUBLIC include)
ament_target_dependencies(gazebo_ros_test
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "sensor_msgs"
)
ament_export_libraries(gazebo_ros_test)
#####################################
add_library(marsrover_control SHARED
  src/marsrover_control.cpp
)
target_include_directories(marsrover_control PUBLIC include)
ament_target_dependencies(marsrover_control
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "sensor_msgs"
)
ament_export_libraries(marsrover_control)
####################################################
add_library(gazebo_no_physics SHARED
  src/gazebo_no_physics.cpp
)
target_include_directories(gazebo_no_physics PUBLIC include)
ament_target_dependencies(gazebo_no_physics
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "sensor_msgs"
)
ament_export_libraries(gazebo_no_physics)

install(TARGETS  gazebo_ros_boom template
gazebo_no_physics
gazebo_ros_test
marsrover_control
gazebo_link_attacher
gazebo_ros_force_mode
gazebo_ros_state_mode 
gazebo_ros_odom
gazebo_ros_3d_move_mode
ball_shooter_plugin
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)




ament_package()
