from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Declare a command-line argument "cmd_line_parameter"
    cmd_line_parameter = DeclareLaunchArgument(
        "cmd_line_parameter",  # Name of the argument
        default_value="default_value",  # Default value
        description="A parameter from the command line.",  # Description of the parameter
    )

    param_file = PathJoinSubstitution(
        [FindPackageShare("parameter_demo"), "config", "params.yaml"]
    )
    param_demo = Node(
        package="parameter_demo",
        executable="param_demo",
        parameters=[
            {"cmd_line_parameter": LaunchConfiguration("cmd_line_parameter")},
            param_file,
        ],
    )

    ld.add_action(param_demo)
    ld.add_action(cmd_line_parameter)
    return ld
