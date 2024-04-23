from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    # Path to the empty_world.launch.py file within the turtlebot3_gazebo package
    include_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("turtlebot3_gazebo"), "/launch", "/empty_world.launch.py"]
        )
    )
    
    srv_client_demo = Node(
        package='interface_demo',        # Name of the package where the node is located
        executable='srv_client_demo',        # Name of the executable (node) to run
        output='screen',            # Direct the output to the screen
    )
    
    srv_server_demo = Node(
        package='interface_demo',        # Name of the package where the node is located
        executable='srv_server_demo',        # Name of the executable (node) to run
        output='screen',            # Direct the output to the screen
    )
    
    msg_demo= Node(
        package='interface_demo',        # Name of the package where the node is located
        executable='msg_demo',        # Name of the executable (node) to run
        output='screen',            # Direct the output to the screen
    )

    ld.add_action(include_empty_world)  # Ad a node action to the object
    ld.add_action(srv_client_demo)  # Ad a node action to the object
    ld.add_action(srv_server_demo)  # Ad a node action to the object
    ld.add_action(msg_demo)  # Ad a node action to the object
    return ld
