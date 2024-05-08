import os
import yaml
import rclpy.logging
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Set the path to this package.
    pkg_share = FindPackageShare(package="final_project").find("final_project")

    # Set the path to the world file

    user_config_path = os.path.join(pkg_share, "config", "sensors.yaml")

    if not os.path.exists(user_config_path):
        rclpy.logging.get_logger("Launch File").fatal(
            f"Sensor configuration 'sensors.yaml' not found in config directory: {user_config_path}."
        )
        exit()

    rviz_config_file = os.path.join(pkg_share, "config", "frame.rviz")

    # Sensor TF
    sensor_tf_broadcaster = Node(
        package="final_project",
        executable="sensor_tf_broadcaster.py",
        output="screen",
        arguments=[user_config_path],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Environment Startup
    environment_startup = Node(
        package="final_project",
        executable="environment_startup_node.py",
        output="screen",
        parameters=[
            {"user_config_path": user_config_path},
            {"use_sim_time": True},
        ],
    )

    part_spawner_cmd = Node(
        package="final_project", executable="part_spawner.py", output="screen"
    )

    start_aruco_detection_node_cmd = Node(
        package="ros2_aruco", executable="aruco_node", output="screen"
    )

    # static_broadcaster = Node(
    #     package='final_project',
    #     executable='object_tf_broadcaster.py',
    #     output='screen')

    static_transform_cmd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0",  # x
            "0",  # y
            "0",  # z
            "0",  # r
            "0",  # p
            "0",  # y
            "world",  # frame_id
            "map",  # child_frame_id
        ],
    )

    multi_robots_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "multi_robots.launch.py")
        ),
    )

    follower_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "start_follower.launch.py")
        ),
    )
    leader_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "start_leader.launch.py")
        ),
    )

    # # Publish static transforms from leader to base frame
    # transform1 = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="leader_to_base_tf",
    #     arguments=["x", "y", "z", "roll", "pitch", "yaw", "leader", "base_footprint"],
    #     output="screen",
    # )

    # # Configure the TF buffer to use the /tf topic
    # buffer_cmd = Node(
    #     package="tf2_ros",
    #     executable="tf2_buffer_server",
    #     name="tf2_buffer_server",
    #     output="screen",
    #     parameters=[
    #         {"use_tf_static": False},  # Subscribe to /tf instead of /tf_static
    #         {"subscribe_timeout": 10.0},  # Adjust as needed
    #     ],
    # )

    # # Republish the transformed /tf messages to the /tf topic
    # relay_cmd = Node(
    #     package="tf2_ros",
    #     executable="tf2_relay",
    #     name="tf2_relay",
    #     output="screen",
    # )

    nodes_to_start = [
        # environment_startup,
        # part_spawner_cmd,
        # sensor_tf_broadcaster,
        # static_transform_cmd,
        multi_robots_cmd,
        # follower_cmd,
        # leader_cmd,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "sensor_config",
            default_value="sensors",
            description="name of user configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Whether to start RViz",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
