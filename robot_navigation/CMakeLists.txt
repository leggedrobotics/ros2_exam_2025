cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set compile commands to ON
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the build type if it isn't already
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Add executable for dummy_localization
add_executable(dummy_localization src/DummyLocalization.cpp)
ament_target_dependencies(dummy_localization
  rclcpp
  tf2_ros
  geometry_msgs
)

# Add library for simple_occupancy_layer
add_library(simple_occupancy_layer_lib SHARED
  src/UpdateOccupancy.cpp)
ament_target_dependencies(simple_occupancy_layer_lib
  rclcpp
  tf2_ros
  geometry_msgs
  laser_geometry
  sensor_msgs
  nav2_costmap_2d
  tf2_sensor_msgs
)

# Add library for waypoint_action_client
add_executable(follow_waypoints_action_client
  src/FollowWaypointsActionClient.cpp)
ament_target_dependencies(follow_waypoints_action_client
  nav2_msgs
  rclcpp
  rclcpp_action
  tf2
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(follow_waypoints_action_client
  yaml-cpp
)

# Export plugin description file
pluginlib_export_plugin_description_file(nav2_costmap_2d simple_occupancy_layer.xml)


# Install the binary file
set(BINARY_PATH "")

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  # If the architecture is x86, use the x86 binary
  set(BINARY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/install/x86/custom_map_saver_server")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  # If the architecture is ARM, use the ARM binary
  set(BINARY_PATH "${CMAKE_CURRENT_SOURCE_DIR}/install/arm/custom_map_saver_server")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Install the binary file for the ROS 2 workspace
install(PROGRAMS ${BINARY_PATH}
  DESTINATION lib/${PROJECT_NAME})

# Install targets
install(TARGETS
  dummy_localization
  follow_waypoints_action_client
  simple_occupancy_layer_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config files, and maps
install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()