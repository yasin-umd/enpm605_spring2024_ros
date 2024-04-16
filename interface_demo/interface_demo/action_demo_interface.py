
import rclpy
import math
from rclpy.action import ActionServer
from rclpy.node import Node
from interface_demo_msgs.action import RobotTarget
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from transformations import euler_from_quaternion


class GoalDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        self._action_server = ActionServer(
            self, RobotTarget, "navigate_to_goal", self.action_server_cb
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._feedback_pub = self.create_publisher(
            Point, "distance_to_goal", 10
        )
        self._odometry_sub = self.create_subscription(
            Odometry, "odom", self.odometry_cb, 10
        )
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._goal_reached = False

        # Create an action client

        self._action_client = ActionClient(self, RobotTarget, "navigate_to_goal")
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Action server 'navigate_to_goal' is not available, waiting...")
        self.send_goal()

    def odometry_cb(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        
        # Retrieve the yaw
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        _, _, self._robot_yaw = euler_from_quaternion(quaternion)

    def action_server_cb(self, goal_handle):
        target = goal_handle.request.target
        
        self._goal_x = target.x
        self._goal_y = target.y
        self._goal_reached = False

        
        while not self._goal_reached:
            # Compute distance to goal
            distance_to_goal = math.sqrt(
                (self._goal_x - self._robot_x) ** 2 + (self._goal_y - self._robot_y) ** 2
            )

            # Compute angle to goal
            angle_to_goal = math.atan2(
                self._goal_y - self._robot_y, self._goal_x - self._robot_x
            )
            
            # Make the robot rotate the correct direction
            if angle_to_goal < 0:
                angle_to_goal = 2 * math.pi + angle_to_goal
            
            # compute relative orientation between robot and goal
            w = angle_to_goal - self._robot_yaw
            if w > math.pi:
                w = w - 2 * math.pi

            # Proportional control for angular velocity
            # Proportional gain = 0.5
            angular_velocity = 0.5 * w
            
            if angular_velocity > 0:
                angular_velocity = min(angular_velocity, 1.5)
            else:
                angular_velocity = max(angular_velocity, -1.5)

            # Proportional control for linear velocity
            # Proportional gain = 0.5
            linear_velocity = min(0.5 * distance_to_goal, 0.6)

            # Publish velocity command
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self._cmd_vel_pub.publish(twist)

            # Publish distance to goal as feedback
            feedback = Point()
            feedback.x = self._robot_x
            feedback.y = self._robot_y
            self._feedback_pub.publish(feedback)

            # Check if goal is reached
            if distance_to_goal < 0.1:  # Threshold for reaching goal
                self._goal_reached = True

        # Once goal is reached, send result
        result = RobotTarget.Result(success=True, message="Goal reached")
        goal_handle.succeed(result)
        
    def send_goal(self):
        goal_msg = RobotTarget.Goal()
        goal_msg.target.x = 1.0
        goal_msg.target.y = 1.0
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback: {feedback_msg}")
        
        
def main(args=None):
    rclpy.init(args=args)
    node = GoalDemoInterface("action_demo")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()