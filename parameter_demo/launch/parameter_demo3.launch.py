from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Declare a command-line argument "cli_arg"
    cli_arg = DeclareLaunchArgument(
        "cli_arg",  # Name of the argument
        default_value="default_value",  # Default value
        description="A parameter from the command line.",  # Description of the parameter
    )

    # Path to the parameter file
    param_file = PathJoinSubstitution(
        [FindPackageShare("parameter_demo"), "config", "params.yaml"]
    )
    
    param_demo = Node(
        package="parameter_demo",
        executable="param_demo",
        parameters=[
            {"cli_arg": LaunchConfiguration("cli_arg")},
            param_file,
        ],
    )

    ld.add_action(cli_arg)
    ld.add_action(param_demo)
    return ld
