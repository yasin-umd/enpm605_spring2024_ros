# import rclpy
# import math
# from rclpy.action import ActionServer
# from rclpy.node import Node
# from interface_demo_msgs.action import RobotTarget, RobotTarget_Goal, RobotTarget_Result, RobotTarget_Feedback
# from geometry_msgs.msg import Twist, Point
# from nav_msgs.msg import Odometry
# from rclpy.action import ActionClient
# from rclpy.executors import MultiThreadedExecutor
# from tf_transformations import euler_from_quaternion

# # import MutuallyExclusiveCallbackGroup
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


# class GoalDemoInterface(Node):
#     def __init__(self, node_name):
#         super().__init__(name=node_name)

#         # Action server
#         self._action_server = ActionServer(
#             self,   # Node
#             RobotTarget,    # Action type
#             'reach_target', # Action server name
#             self.action_server_cb, # Callback function
#             goal_callback=self.handle_goal, # Goal callback
#             cancel_callback=self.handle_cancel # Cancel callback
#         )

#         # Publisher to publish velocity commands
#         self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
#         # Publisher to publish distance to goal
#         self._feedback_pub = self.create_publisher(RobotTarget_Feedback, "distance_to_goal", 10)
#         # Subscriber to get odometry data
#         self._odometry_cbg = MutuallyExclusiveCallbackGroup()
#         self._odometry_sub = self.create_subscription(
#             Odometry, "odom", callback=self.odometry_cb, qos_profile=10, callback_group=self._odometry_cbg
#         )

#         # Initialize variables
#         self._goal_x = 0.0
#         self._goal_y = 0.0
#         self._robot_x = 0.0
#         self._robot_y = 0.0
#         self._robot_yaw = 0.0
#         # Flag to check if goal is reached
#         self._goal_reached = False

#         # Action client
#         self._action_client = ActionClient(self, RobotTarget, "reach_target")
#         # Wait for action server "reach_target" to be available
#         while not self._action_client.wait_for_server(timeout_sec=1.0):
#             self.get_logger().info(
#                 "Action server 'navigate_to_goal' is not available, waiting..."
#             )

#         # Send goal
#         self.send_goal()

#     def odometry_cb(self, msg: Odometry):
#         """
#         Callback function for odometry data

#         Args:
#             msg (Odometry): Odometry message
#         """

#         # Retrieve current position
#         self._robot_x = msg.pose.pose.position.x
#         self._robot_y = msg.pose.pose.position.y

#         # Retrieve the yaw
#         quaternion = (
#             msg.pose.pose.orientation.x,
#             msg.pose.pose.orientation.y,
#             msg.pose.pose.orientation.z,
#             msg.pose.pose.orientation.w,
#         )
#         _, _, self._robot_yaw = euler_from_quaternion(quaternion)

#     def action_server_cb(self, goal_handle):
#         """
#         Callback function for action server.
#         This callback is executed when a new goal is received.

#         Args:
#             goal_handle (): Request sent by the client
#         """
#         self.get_logger().info('Executing goal...')
#         feedback_msg = RobotTarget.Feedback()

#         target = goal_handle.request.target

#         self._goal_x = target.x
#         self._goal_y = target.y
#         self._goal_reached = False

#         while not self._goal_reached:
#             # Compute distance to goal
#             distance_to_goal = math.sqrt(
#                 (self._goal_x - self._robot_x) ** 2
#                 + (self._goal_y - self._robot_y) ** 2
#             )

#             # Compute angle to goal
#             angle_to_goal = math.atan2(
#                 self._goal_y - self._robot_y, self._goal_x - self._robot_x
#             )

#             # Make the robot rotate the correct direction
#             if angle_to_goal < 0:
#                 angle_to_goal = 2 * math.pi + angle_to_goal

#             # compute relative orientation between robot and goal
#             w = angle_to_goal - self._robot_yaw
#             if w > math.pi:
#                 w = w - 2 * math.pi

#             # Proportional control for angular velocity
#             # Proportional gain = 0.5
#             angular_velocity = 0.5 * w

#             # turtlebot max angular velocity is 1.82 rad/s
#             if angular_velocity > 0:
#                 angular_velocity = min(angular_velocity, 1.82)
#             else:
#                 angular_velocity = max(angular_velocity, -1.82)

#             # Proportional control for linear velocity
#             # Proportional gain = 0.5
#             linear_velocity = min(0.5 * distance_to_goal, 0.26)

#             # Publish velocity command
#             twist = Twist()
#             twist.linear.x = linear_velocity
#             twist.angular.z = angular_velocity
#             self._cmd_vel_pub.publish(twist)

#             # Publish distance to goal as feedback
#             feedback_msg.distance_to_goal = distance_to_goal
#             goal_handle.publish_feedback(feedback_msg)

#             # Check if goal is reached
#             if distance_to_goal < 0.1:  # Threshold for reaching goal
#                 self._goal_reached = True
#                 # stop the robot
#                 twist = Twist()
#                 self._cmd_vel_pub.publish(twist)
#         # Once goal is reached, send result
#         result = RobotTarget.Result()
#         result.success = True
#         result.message = "Goal reached"
#         goal_handle.succeed(result)
#         return RobotTarget.Result(success=True, message="Goal reached")

#     def send_goal(self):
#         goal_msg = RobotTarget.Goal()
#         goal_msg.target.x = 4.0
#         goal_msg.target.y = 4.0
#         self._action_client.send_goal_async(goal_msg)

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f"Feedback: {feedback_msg}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = GoalDemoInterface("action_demo")
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)
#     try:
#         executor.spin()
#     except KeyboardInterrupt:
#         node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
#     except Exception as e:
#         node.get_logger().error(f"Unexpected error: {str(e)}")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile
from interface_demo_msgs.action import (
    RobotTarget,
)  # Assuming the package name is interface_msgs
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
import math


class ActionServerDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)

        self._action_server = ActionServer(
            node=self,  # Node
            action_type=RobotTarget,  # Action type
            action_name="reach_target",  # Action server name
            execute_callback=self.action_server_cb,  # Callback function
            goal_callback=self.handle_goal,  # Goal callback
            cancel_callback=self.handle_cancel,  # Cancel callback
        )

        # Publisher to publish velocity commands
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriber to get odometry data
        # We are using a MutuallyExclusiveCallbackGroup to ensure
        # that the callback is executed in a separate thread from the main thread
        self._odometry_cbg = MutuallyExclusiveCallbackGroup()
        self._odometry_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odometry_cb,
            10,
            callback_group=self._odometry_cbg,
        )

        # Initialize variables
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._goal_reached = False

        # Proportional control gains
        self._kl = 0.5 # Linear velocity gain
        self._ka = 0.8 # Angular velocity gain

        # Print the server is available
        output = "\n============================================="
        output += "\nAction Server 'reach_target' is available"
        output += "\n============================================="

        self.get_logger().info(output)

    def handle_goal(self, goal_request):
        self.get_logger().info("Received new goal request")
        self._goal_x = goal_request.target.x
        self._goal_y = goal_request.target.y
        self._goal_reached = False
        # Let the action client know that the goal is accepted
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle):
        self.get_logger().info("Received cancel request")
        # Set the goal as canceled
        return CancelResponse.ACCEPT

    def odometry_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        _, _, self._robot_yaw = euler_from_quaternion(quaternion)

    def action_server_cb(self, goal_handle):
        self.get_logger().info(f"Executing goal: ({self._goal_x},{self._goal_y})")
        rate = self.create_rate(1)  # 1 Hz

        while not self._goal_reached:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = RobotTarget.Result()
                result.success = False
                result.message = "Goal canceled"
                return result
            
            distance_to_goal = math.sqrt(
                (self._goal_x - self._robot_x) ** 2
                + (self._goal_y - self._robot_y) ** 2
            )

            # If robot is within 2 cm of the goal
            if distance_to_goal < 0.02:
                self._goal_reached = True
                twist = Twist()
                self._cmd_vel_pub.publish(twist)
                self.get_logger().info("Goal reached.")
                try:
                    # Prepare the result message
                    goal_handle.succeed()
                    result = RobotTarget.Result()
                    result.success = True
                    result.message = "Goal reached"
                    return result
                except Exception as e:
                    self.get_logger().error(f"Error processing goal: {str(e)}")
                    goal_handle.abort()
                    result = RobotTarget.Result()
                    result.success = False
                    result.message = "Exception occurred"
                    return result
            else:
                # Continue executing motion control
                self.adjust_robot_motion(distance_to_goal, goal_handle)
            rate.sleep()

        # Log if exited loop without reaching the goal (should not happen in normal operation)
        # if not self._goal_reached:
        #     self.get_logger().warn("Exited loop without reaching goal.")

    def adjust_robot_motion(self, distance_to_goal, goal_handle):
        angle_to_goal = math.atan2(
            self._goal_y - self._robot_y, self._goal_x - self._robot_x
        )
        angular_velocity = self.calculate_angular_velocity(angle_to_goal)
        linear_velocity = min(
            self._kl * distance_to_goal, 0.26
        )  # Proportional control for linear velocity

        # Publish velocity command
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self._cmd_vel_pub.publish(twist)

        # Publish distance to goal as feedback
        feedback_msg = RobotTarget.Feedback()
        feedback_msg.distance_to_goal = distance_to_goal
        goal_handle.publish_feedback(feedback_msg)

    def calculate_angular_velocity(self, angle_to_goal):
        # Adjust angle to ensure proper rotation direction
        if angle_to_goal < 0:
            angle_to_goal += 2 * math.pi

        # Compute relative orientation between robot and goal
        w = angle_to_goal - self._robot_yaw
        if w > math.pi:
            w -= 2 * math.pi
        elif w < -math.pi:
            w += 2 * math.pi

        # Proportional control for angular velocity, with limit for turtlebot max angular velocity
        angular_velocity = self._ka * w
        return max(min(angular_velocity, 1.82), -1.82)


def main(args=None):
    rclpy.init(args=args)
    node = ActionServerDemoInterface("action_server_demo")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        rclpy.shutdown()
