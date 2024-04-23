from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    param_demo = Node(
        package="parameter_demo",
        executable="param_demo",
        output="screen",
    )

    ld.add_action(param_demo)  # Ad a node action to the object
    return ld
