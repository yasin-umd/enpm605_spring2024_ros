from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter


class ParameterDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Parameter descriptor
        param_descriptor = ParameterDescriptor(
            description="An integer parameter with constraints",
            type=ParameterType.PARAMETER_INTEGER,
            integer_range=[IntegerRange(from_value=1, to_value=10, step=1)],
            dynamic_typing=True,
        )

        # Declare the parameter with a default value and a descriptor
        self.declare_parameter("my_parameter", 5, descriptor=param_descriptor)

        # Getting the parameter
        self._my_parameter = (
            self.get_parameter("my_parameter").get_parameter_value().integer_value
        )
        self.get_logger().info(f"Initial value of my_parameter: {self._my_parameter}")

        # Setting the parameter
        self.set_parameters(
            [Parameter("my_parameter", rclpy.Parameter.Type.INTEGER, 7)]
        )
        self._my_parameter = (
            self.get_parameter("my_parameter").get_parameter_value().integer_value
        )
        self.get_logger().info(f"New value of my_parameter: {self._my_parameter}")


def main(args=None):
    rclpy.init(args=args)
    node = ParameterDemoInterface("parameter_demo")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        rclpy.shutdown()
