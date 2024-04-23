from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    param_demo = Node(
        package="parameter_demo",
        executable="param_demo",
        parameters=[{"my_param": 10}],
        output="screen",
    )

    ld.add_action(param_demo)  # Ad a node action to the object
    return ld
