import rclpy
from first_pkg.qos_interface import QoSPublisherInterface


def main(args=None):
    # initialize ROS client library for Python
    # initialize ROS communications
    rclpy.init(args=args)
    # instantiate a node
    node = QoSPublisherInterface("qos_publisher_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        # clean up the node
        rclpy.shutdown()
