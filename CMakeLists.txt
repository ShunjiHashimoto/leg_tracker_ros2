cmake_minimum_required(VERSION 3.8)
project(leg_tracker_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)

# Include our own headers
include_directories(include)

# Generate msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegArray.msg"
  "msg/Leg.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Create node
add_executable(occupancy_grid_mapping src/local_occupancy_grid_mapping.cpp)
rosidl_target_interfaces(occupancy_grid_mapping ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(occupancy_grid_mapping 
  rclcpp 
  sensor_msgs 
  nav_msgs 
  tf2_ros 
  visualization_msgs 
  geometry_msgs
  tf2_geometry_msgs 
  tf2
  std_msgs
  )

# Install our headers
install(
  DIRECTORY include/
  DESTINATION include
)
 
install(
  TARGETS occupancy_grid_mapping 
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
