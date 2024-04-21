import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from interface_demo_msgs.action import RobotTarget


class ActionClientDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self._action_client = ActionClient(self, RobotTarget, "reach_target")
        self.wait_for_action_server()

    def wait_for_action_server(self):
        """
        Wait for the action server to be available before sending a goal
        """
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for the action server 'reach_target'...")
        self.get_logger().info("Action server is now available.")
        self.send_goal(2.0, 1.5)

    def send_goal(self, goal_x=4.0, goal_y=4.0):
        """
        Send a goal to the action server
        """
        goal_msg = RobotTarget.Goal()
        # Set up goal message details
        goal_msg.target.x = goal_x
        goal_msg.target.y = goal_y
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        # It's good practice to handle the future result even if we don't expect to use it
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by server 😢')
            return
        
        self.get_logger().info('Goal accepted by server 😊')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info(f'Goal completed - Success: {result.success}, Message: "{result.message}"')
        else:
            self.get_logger().info("Goal failed 😢")

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(
            f"Received feedback: {feedback_msg.feedback.distance_to_goal}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ActionClientDemoInterface("action_client_demo")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        rclpy.shutdown()
