#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ConfigPrinter(Node):
    def __init__(self):
        super().__init__('config_printer')
        self._map_dir = self.declare_parameter('map', 'default_value').get_parameter_value().string_value
        self.timer = self.create_timer(1, self.print_config)

    def print_config(self):
        self._map_dir = self.get_parameter('map').get_parameter_value().string_value
        self.get_logger().info(f'Map Directory: {self._map_dir}')

def main(args=None):
    rclpy.init(args=args)
    node = ConfigPrinter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()