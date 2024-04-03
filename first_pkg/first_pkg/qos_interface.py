from rclpy.node import Node
from std_msgs.msg import Int64
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)


class QoSPublisherInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._counter = 0
        self._timer_interval = 1
        self._message = Int64()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )
        # publisher object
        self._publisher = self.create_publisher(Int64, "counter", qos_profile)

        # timer object
        self._timer = self.create_timer(self._timer_interval, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(str(self._counter))
        self._message.data = self._counter
        self._publisher.publish(self._message)
        self._counter += 1


class QoSSubscriberInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
        )
        # subscriber object
        self._subscriber = self.create_subscription(
            Int64, "counter", self.subscriber_callback, qos_profile
        )

    def subscriber_callback(self, msg: Int64):
        self.get_logger().info(f"Received: {msg.data}")
