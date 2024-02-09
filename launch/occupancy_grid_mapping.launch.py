from os.path import join
import os
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    map_node = Node(
        package = 'leg_tracker_ros2',
        executable = 'occupancy_grid_mapping', 
        output='screen',
        emulate_tty=True,
        arguments=[('__log_level:=debug')],
        parameters=[
            {'local_map_topic':'local_map'},
            {'local_map_resolution':0.05},
            {'use_scan_header_stamp_for_tfs':True},
            {'fixed_frame':'base_link'},  
        ]
    )
    return LaunchDescription([
        map_node, 
    ])
    