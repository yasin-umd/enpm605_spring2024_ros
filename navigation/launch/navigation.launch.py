from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    leader_cmd = Node(
        package="navigation",
        executable="leader_navigator",
        output="screen",
    )
    
    follower_cmd = Node(
        package="navigation",
        executable="follower_navigator",
        output="screen",
    )

    ld.add_action(leader_cmd)  # Ad a node action to the object
    ld.add_action(follower_cmd)  # Ad a node action to the object
    return ld