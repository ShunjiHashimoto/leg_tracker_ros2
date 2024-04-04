#!/usr/bin/python3

import os
from os.path import join
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

leg_detector_path = get_package_share_directory('leg_tracker_ros2')
rosbag_path = leg_detector_path + "/rosbag/demos/demo_stationary_simple_environment"
rviz2_config_path = leg_detector_path + "/rosbag/demos/rviz/demo_stationary_simple_environment.rviz"
forest_file_path = leg_detector_path + "/config/trained_leg_detector_res=0.33.yaml"

def generate_launch_description():

    ld = LaunchDescription([

        # Launching Rosbag node
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-s', 'sqlite3', rosbag_path],
            output='screen'
        ),

        # Launching RVIZ2
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz2_config_path],
            output='screen'
        )
    ])

    # Launching detect_leg_clusters node
    detect_leg_clusters_node = Node(
            package="leg_tracker_ros2",
            executable="detect_leg_clusters",
            name="detect_leg_clusters",
            parameters= [
                {"forest_file" : forest_file_path},
                {"scan_topic" : "/scan"},
                {"fixed_frame" : "laser"},
            ]
    )

    # Launching joint_leg_tracker node
    joint_leg_tracker_node = Node(
        package="leg_tracker_ros2",
        executable="joint_leg_tracker.py",
        name="joint_leg_tracker",
        parameters=[
            {"scan_topic" : "/scan"},
            {"fixed_frame" : "laser"},
            {"scan_frequency" : 10}
        ]    
    )

    # Launching inflated_human_scan node
    # inflated_human_scan_node = Node(
    #     package="leg_detector",
    #     executable="inflated_human_scan",
    #     name="inflated_human_scan",
    #     parameters=[
    #         {"inflation_radius" : 1.0}
    #     ]
    # )
        
    # Launching local_occupancy_grid_mapping node
    local_occupancy_grid_mapping_node = Node(
        package="leg_tracker_ros2",
        executable="occupancy_grid_mapping",
        name="occupancy_grid_mapping",
        parameters=[
            {"scan_topic" : "/scan"},
            {"fixed_frame" : "laser"},
        ]    
    )

    # Include URG Node2 Launch File
    pkg_prefix = get_package_share_directory('urg_node2')
    launch_path = join(pkg_prefix, 'launch/urg_node2.launch.py')
    urg_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_path))

    ld.add_action(detect_leg_clusters_node)
    ld.add_action(joint_leg_tracker_node)
    # ld.add_action(inflated_human_scan_node)
    ld.add_action(local_occupancy_grid_mapping_node)
    ld.add_action(urg_node)

    return ld 
 
