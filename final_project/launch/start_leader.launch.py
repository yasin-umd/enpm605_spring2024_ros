#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node



    
def generate_launch_description():

    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_final_project = get_package_share_directory("final_project")
    pkg_tb3_nav2 = get_package_share_directory("turtlebot3_navigation2")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="2.0")
    y_pose = LaunchConfiguration("y_pose", default="1.0")
    rviz = LaunchConfiguration("rviz", default="false")

    world = os.path.join(
        pkg_final_project,
        "world",
        "final.world",
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    # --- Start Nav2 stack ---

    # Map file
    map_file = os.path.join(pkg_final_project, "map", "final_map.yaml")
    # Nav2 nodes
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_nav2, "launch", "navigation2.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time, "map": map_file}.items(),
    )
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    # --- Start RViz ---
    # RViz config file
    # rviz_config_file = os.path.join(
    #     pkg_navigation_demo, "config", "navigation_demo.rviz"
    # )
    # # start rviz only if the argument is set to true
    # rviz_cmd = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[{"use_sim_time": use_sim_time}],
    #     condition=IfCondition(rviz),
    # )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(start_nav2_cmd)

    return ld
