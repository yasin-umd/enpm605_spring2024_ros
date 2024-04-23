from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterEvent


class ParameterDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        # timer to display parameter value every 1
        self._timer = self.create_timer(1, self.timer_callback)

        # ---------------------- Declaring Parameters ---------------------- #
        # Parameter descriptor
        param_descriptor = ParameterDescriptor(
            description="An integer parameter with constraints",
            type=ParameterType.PARAMETER_INTEGER,
            integer_range=[IntegerRange(from_value=1, to_value=10, step=1)],
            dynamic_typing=True,
        )

        # Declare the parameter with a default value and a descriptor
        self.declare_parameter("my_param", 5, descriptor=param_descriptor)

        # ---------------------- Retrieving Parameters ---------------------- #
        # Getting the parameter
        self._my_param = (
            self.get_parameter("my_param").get_parameter_value().integer_value
        )
        self.get_logger().info(f"Initial value of my_parameter: {self._my_param}")

        # ---------------------- Setting Parameters ---------------------- #
        # # Setting the parameter
        # self.set_parameters([Parameter("my_param", rclpy.Parameter.Type.INTEGER, 7)])
        # self._my_param = (
        #     self.get_parameter("my_param").get_parameter_value().integer_value
        # )
        # self.get_logger().info(f"New value of my_param: {self._my_param}")

        # ---------------------- Parameter Changes for Current Node ---------------------- #
        # callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ---------------------- Parameter Events for All Nodes ---------------------- #
        # subscribe to /parameter_events topic
        self._sub_parameter_events = self.create_subscription(
            ParameterEvent, "/parameter_events", self.parameter_event_callback, 10
        )

    def parameter_callback(self, params):
        """
        Callback function for parameter changes for the current node
        """
        success = False
        for param in params:
            if param.name == "my_param" and param.type_ == Parameter.Type.INTEGER:
                success = True
                # Handle the parameter change (e.g., reconfigure your node)
                self._my_param = param.value
        return SetParametersResult(successful=success)

    def parameter_event_callback(self, event):
        """
        Callback function for parameter events for all nodes
        """
        for changed_parameter in event.changed_parameters:
            if changed_parameter.name == "use_sim_time" and event.node == "/talker":
                new_value = changed_parameter.value.bool_value
                self.get_logger().info(
                    f"'use_sim_time' parameter changed on /talker to: {new_value}"
                )

    def timer_callback(self):
        self._my_param = (
            self.get_parameter("my_param").get_parameter_value().integer_value
        )
        self.get_logger().info(f"Current value of my_param: {self._my_param}")

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
