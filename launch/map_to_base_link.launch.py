import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    current_dir=get_package_share_directory('map_to_base_link')
    ld = LaunchDescription()
    
    launch_turtlebot3_odom = Node(
       package='map_to_base_link',
       executable='map_to_base_link',
       output='screen')
    
    ld.add_action(launch_turtlebot3_odom)
    return ld
    
