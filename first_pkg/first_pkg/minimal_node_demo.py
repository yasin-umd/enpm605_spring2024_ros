import rclpy
from rclpy.node import Node

def main(args=None):
    # initialize ROS client library for Python
    # initialize ROS communications
    rclpy.init(args=args)
    # instantiate a node
    node = Node("minimal_node")
    # print message in the terminal
    node.get_logger().info("Hello World")
    # clean up
    rclpy.shutdown()