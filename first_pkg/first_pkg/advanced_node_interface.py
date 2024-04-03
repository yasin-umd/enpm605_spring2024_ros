from rclpy.node import Node
from std_msgs.msg import Int64


class AdvancedNodeInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._counter = 0
        self._timer_interval = 2
        self._message = Int64()

        # publisher object
        self._publisher = self.create_publisher(Int64, "counter", 10)
        # subscriber object
        self._subscriber = self.create_subscription(
            Int64, "counter", self.subscriber_callback, 10
        )
        
        # timer object
        self._timer = self.create_timer(self._timer_interval, self.timer_callback)

    def subscriber_callback(self, msg: Int64):
        self.get_logger().info(f"Received: {msg.data}")
        
    def timer_callback(self):
        self.get_logger().info(str(self._counter))
        self._message.data = self._counter
        self._publisher.publish(self._message)
        self._counter += 1
