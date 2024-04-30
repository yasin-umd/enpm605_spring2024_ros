#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity
from turtlebot3_gazebo.spawn_params import SpawnParams
from turtlebot3_gazebo.spawn_params import PartSpawnParams


class PartSpawner(Node):
    def __init__(self):
        super().__init__('part_spawner')

        # Create service client to spawn objects into gazebo
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

    def spawn_part(self, name, type, color, xyz, rpy):
        params = PartSpawnParams(name, type, color, xyz=xyz, rpy=rpy)

        self.spawn_entity(params)

    def spawn_entity(self, params: SpawnParams) -> bool:
        self.spawn_client.wait_for_service()

        self.get_logger().info(f'Spawning: {params.name}')

        req = SpawnEntity.Request()

        req.name = params.name
        req.xml = params.xml
        req.initial_pose = params.initial_pose
        req.robot_namespace = params.robot_namespace
        req.reference_frame = params.reference_frame

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success


if __name__ == "__main__":
    rclpy.init()

    part_spawner = PartSpawner()
    part_spawner.spawn_part("blue_battery_99", "battery", "blue", [
                            -0.569359, 1.834770, 0.35], [0, 0, 0.78539816339])
    part_spawner.spawn_part("green_battery_99", "battery",
                            "green", [-1.047505, -0.939857, 0.35], [0, -0.33, 1.57])
    part_spawner.spawn_part("yellow_orange_99", "battery",
                            "orange", [-2.277804, -3.026862, 0.35], [0, 0.33, 1.57])

    part_spawner.destroy_node()
    rclpy.shutdown()
