import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #pkg_path = get_package_share_directory("teamx_challenge")
    #config_file = os.path.join(pkg_path,"config","params.yaml")
    
    action_vision = Node(
        package="teamx_challenge",
        executable="vision_node",
        name="vision_node",
        #parameters=[config_file],  # YAML
        output="screen"
    )
    action_shooter = Node(
        package="teamx_challenge",
        executable="shooter_node",
        name="shooter_node",
        #parameters=[config_file],  # YAML
        output="screen"
    )
    
    launch_description = LaunchDescription([
        action_vision, 
        action_shooter  
    ])
    return launch_description