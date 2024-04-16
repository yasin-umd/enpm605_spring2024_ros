from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import rclpy
from rclpy.executors import MultiThreadedExecutor


class Color:
    RED = "\033[1;31m"
    GREEN = "\033[1;32m"
    YELLOW = "\033[1;33m"
    BLUE = "\033[1;34m"
    RESET = "\033[0m"


class SingleThreadedExecutorInterface(Node):
    """
    Class to demonstrate the use of a single-threaded executor.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback)

        self.get_logger().info(f"{node_name} initialized")

    def timer1_callback(self):
        """
        Callback function for timer1
        """
        self.get_logger().info(Color.YELLOW + "Timer1 callback" + Color.RESET)

    def timer2_callback(self):
        """
        Callback function for timer2
        """
        self.get_logger().info(Color.BLUE + "Timer2 callback" + Color.RESET)

    def timer3_callback(self):
        """
        Callback function for timer3
        """
        self.get_logger().info(Color.GREEN + "Timer3 callback" + Color.RESET)

    def timer4_callback(self):
        """
        Callback function for timer4
        """
        self.get_logger().info(Color.RED + "Timer4 callback" + Color.RESET)


class DualMutuallyExclusiveInterface(Node):
    """
    Class to demonstrate the use of a dual mutually exclusive callback groups.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        group1 = MutuallyExclusiveCallbackGroup()
        group2 = MutuallyExclusiveCallbackGroup()

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback, callback_group=group1)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback, callback_group=group1)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback, callback_group=group2)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback, callback_group=group2)

        self.get_logger().info(f"{node_name} initialized")

    def timer1_callback(self):
        """
        Callback function for timer1
        """
        self.get_logger().info(Color.YELLOW + "Timer1 callback" + Color.RESET)

    def timer2_callback(self):
        """
        Callback function for timer2
        """
        self.get_logger().info(Color.BLUE + "Timer2 callback" + Color.RESET)
        # while True:
        #     pass

    def timer3_callback(self):
        """
        Callback function for timer3
        """
        self.get_logger().info(Color.GREEN + "Timer3 callback" + Color.RESET)

    def timer4_callback(self):
        """
        Callback function for timer4
        """
        self.get_logger().info(Color.RED + "Timer4 callback" + Color.RESET)


class ExclusiveReentrantInterface(Node):
    """
    Class to demonstrate the use of an exclusive and reentrant callback groups.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        group1 = MutuallyExclusiveCallbackGroup()
        group2 = ReentrantCallbackGroup()

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback, callback_group=group1)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback, callback_group=group1)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback, callback_group=group2)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback, callback_group=group2)

        self.get_logger().info(f"{node_name} initialized")

    def timer1_callback(self):
        """
        Callback function for timer1
        """
        self.get_logger().info(Color.YELLOW + "Timer1 callback" + Color.RESET)

    def timer2_callback(self):
        """
        Callback function for timer2
        """
        self.get_logger().info(Color.BLUE + "Timer2 callback" + Color.RESET)
        # while True:
        #     pass

    def timer3_callback(self):
        """
        Callback function for timer3
        """
        self.get_logger().info(Color.GREEN + "Timer3 callback" + Color.RESET)

    def timer4_callback(self):
        """
        Callback function for timer4
        """
        self.get_logger().info(Color.RED + "Timer4 callback" + Color.RESET)


class ReentrantInterface(Node):
    """
    Class to demonstrate the use of a reentrant callback group.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        group = ReentrantCallbackGroup()

        # Timer1
        self._timer1 = self.create_timer(1, self.timer1_callback, callback_group=group)
        # Timer2
        self._timer2 = self.create_timer(1, self.timer2_callback, callback_group=group)
        # Timer3
        self._timer3 = self.create_timer(1, self.timer3_callback, callback_group=group)
        # Timer4
        self._timer4 = self.create_timer(1, self.timer4_callback, callback_group=group)

        self.get_logger().info(f"{node_name} initialized")

    def timer1_callback(self):
        """
        Callback function for timer1
        """
        self.get_logger().info(Color.YELLOW + "Timer1 callback" + Color.RESET)

    def timer2_callback(self):
        """
        Callback function for timer2
        """
        self.get_logger().info(Color.BLUE + "Timer2 callback" + Color.RESET)
        # while True:
        #     pass

    def timer3_callback(self):
        """
        Callback function for timer3
        """
        self.get_logger().info(Color.GREEN + "Timer3 callback" + Color.RESET)

    def timer4_callback(self):
        """
        Callback function for timer4
        """
        self.get_logger().info(Color.RED + "Timer4 callback" + Color.RESET)
        

# Dual mutex demo
def main_dual_mutex(args=None):
    rclpy.init(args=args)
    node = DualMutuallyExclusiveInterface("dual_mutually_exclusive_demo")

    # Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

# Mutex + Reentrant
def main_mutex_reentrant(args=None):
    rclpy.init(args=args)
    node = ExclusiveReentrantInterface("exclusive_reentrant_demo")

    # Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

# Reentrant demo
def main_reentrant(args=None):
    rclpy.init(args=args)
    node = ReentrantInterface("reentrant_demo")

    # Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

# Single threaded demo
def main_singlethreaded(args=None):
    """
    Main function to initialize and run the ROS2 publisher node.

    Args:
        args (list, optional): Command-line arguments passed to the node. Defaults to None.
    """
    rclpy.init(args=args)
    node = SingleThreadedExecutorInterface("single_threaded_executor_demo")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Log a message when the node is manually terminated
        node.get_logger().warn("Keyboard interrupt detected")
    finally:
        # Cleanly destroy the node instance
        node.destroy_node()
        # Shut down the ROS 2 Python client library
        rclpy.shutdown()
