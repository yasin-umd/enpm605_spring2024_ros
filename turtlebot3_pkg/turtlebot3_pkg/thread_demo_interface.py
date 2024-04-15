from rclpy.node import Node
import rclpy

class ThreadDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self._timer1 = self.create_timer(1, self.timer1_cb)
        self._timer2 = self.create_timer(1, self.timer2_cb)
        self._timer3 = self.create_timer(1, self.timer3_cb)

    def timer1_cb(self):
        self.get_logger().info("Timer1")

    def timer2_cb(self):
        self.get_logger().info("Timer2")

    def timer3_cb(self):
        self.get_logger().info("Timer3")

def main(args=None):
    rclpy.init(args=args)

    node = ThreadDemoInterface("thread_demo_interface")

    rclpy.spin(node)

    rclpy.shutdown()