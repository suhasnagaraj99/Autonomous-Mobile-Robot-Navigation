cmake_minimum_required(VERSION 3.8)
project(group5_final)

set(CMAKE_CXX_STANDARD 11) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# add_library(action_client SHARED
#   src/tbot_follow_waypoints.cpp)
# target_include_directories(action_client PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(action_client
#   PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
# ament_target_dependencies(action_client
#   "action_tutorials_interfaces"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components")
# rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
# install(TARGETS
#   action_client
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

include_directories(include/group5_final)

add_executable(tbot_through_poses src/tbot_through_poses.cpp)
ament_target_dependencies(tbot_through_poses rclcpp rclcpp rclcpp_action rclcpp_components nav2_msgs ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(camera1_broadcaster src/camera1_broadcaster.cpp)
ament_target_dependencies(camera1_broadcaster rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(camera2_broadcaster src/camera2_broadcaster.cpp)
ament_target_dependencies(camera2_broadcaster rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(camera3_broadcaster src/camera3_broadcaster.cpp)
ament_target_dependencies(camera3_broadcaster rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(camera4_broadcaster src/camera4_broadcaster.cpp)
ament_target_dependencies(camera4_broadcaster rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(camera5_broadcaster src/camera5_broadcaster.cpp)
ament_target_dependencies(camera5_broadcaster rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(tbot_get_goals src/tbot_get_goals.cpp)
ament_target_dependencies(tbot_get_goals rclcpp ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)

add_executable(tbot_follow_waypoints src/tbot_follow_waypoints.cpp)
ament_target_dependencies(tbot_follow_waypoints rclcpp rclcpp rclcpp_action rclcpp_components nav2_msgs ros2_aruco_interfaces tf2_ros geometry_msgs tf2 tf2_geometry_msgs nav_msgs mage_msgs)


install(TARGETS 
  tbot_through_poses
  tbot_follow_waypoints
  camera1_broadcaster
  camera2_broadcaster
  camera3_broadcaster
  camera4_broadcaster
  camera5_broadcaster
  tbot_get_goals
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config launch DESTINATION share/${PROJECT_NAME})


ament_package()
