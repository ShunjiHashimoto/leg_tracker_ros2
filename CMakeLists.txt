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
find_package(OpenCV REQUIRED)

# Include our own headers
include_directories(include ${OpenCV_INCLUDE_DIR})

# Generate msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegArray.msg"
  "msg/Leg.msg"
  "msg/PersonArray.msg"
  "msg/Person.msg"
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
target_link_libraries(occupancy_grid_mapping laser_processor)

add_executable(detect_leg_clusters src/detect_leg_clusters.cpp)
rosidl_target_interfaces(detect_leg_clusters ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(detect_leg_clusters 
  rclcpp 
  sensor_msgs 
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs 
  )
target_link_libraries(detect_leg_clusters laser_processor)
target_link_libraries(detect_leg_clusters cluster_features)
target_link_libraries(detect_leg_clusters ${OpenCV_LIBRARIES})

add_library(laser_processor SHARED src/laser_processor.cpp)
ament_target_dependencies(laser_processor 
  rclcpp
  sensor_msgs 
  tf2
  tf2_ros
  tf2_geometry_msgs
  )

add_library(cluster_features SHARED src/cluster_features.cpp)
ament_target_dependencies(cluster_features 
  rclcpp
  sensor_msgs 
  tf2
  tf2_ros
  tf2_geometry_msgs
  )
target_link_libraries(cluster_features laser_processor)
target_link_libraries(cluster_features ${OpenCV_LIBRARIES})

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
  TARGETS detect_leg_clusters 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS laser_processor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS cluster_features 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rosbag 
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/joint_leg_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/imu_to_tf_node.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
