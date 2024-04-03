from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    advanced_node = Node(
        package="first_pkg",
        executable="advanced_node",
        name="advanced_node",
        output="screen",
    )
    

    ld.add_action(advanced_node)
    return ld