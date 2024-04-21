import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from interface_demo_msgs.action import RobotTarget


class ActionClientDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self._goal_handle = None
        self._action_client = ActionClient(self, RobotTarget, "reach_target")
        self.wait_for_action_server()
        
        # List of goals to send from a timer
        self._goals = [
            (0.0, 2.0),
            (2.0, 2.0),
            (2.0, 0.0),
        ]
        
        self._timer = self.create_timer(8.0, self.timer_callback)

    def wait_for_action_server(self):
        """
        Wait for the action server to be available before sending a goal
        """
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for the action server 'reach_target'...")
        self.get_logger().info("Action server is now available.")
        # self.send_goal((0.0, 2.0))
        
    def timer_callback(self):
        """
        Send a goal to the action server every 2 seconds
        """
        # If list is not empty
        if self._goals:
            if len(self._goals) == 1:
                self.cancel_goal()
            else:
                self.send_goal(self._goals.pop(0))
            

    def send_goal(self, goal):
        """
        Send a goal to the action server
        """
        goal_msg = RobotTarget.Goal()
        # Set up goal message details
        goal_msg.target.x = goal[0]
        goal_msg.target.y = goal[1]
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        # It's good practice to handle the future result even if we don't expect to use it
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected by server 😢')
        else:   
            self.get_logger().info('Goal accepted by server 😊')
            self.get_result_future = self._goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info(f'Goal completed - Success: {result.success}, Message: "{result.message}"')
        else:
            self.get_logger().info("Goal failed 😢")
            
    def cancel_goal(self):
        if self._goal_handle:
            self.get_logger().info('Cancelling goal')
            # Send a cancel request
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)
            
    def cancel_response_callback(self, future):
        cancel_response = future.result()
        if cancel_response.goals_canceling:
            self.get_logger().info('Goal successfully cancelled')
        else:
            self.get_logger().info('Goal cancel request failed')

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
