from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from navigation.follower_navigator_interface import FollowerNavigatorInterface, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations


class NavigationDemoInterface(Node):
    """
    Example of a class that uses the BasicNavigator class to navigate the robot.
    """

    def __init__(self, node_name, namespace=""):
        super().__init__(node_name)

        # Since we are using Gazebo, we need to set the use_sim_time parameter to True
        self._sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([self._sim_time])
        
        # Navigator
        self._navigator = FollowerNavigatorInterface()
        # Initial pose
        self._initial_pose = PoseStamped()

        # Set the initial pose of the robot
        self.localize()
        
        # subscriber to amcl_pose topic in a different ROS_DOMAIN_ID
        # self._amcl_pose_sub = self.create_subscription(
        #     PoseStamped,
        #     "/amcl_pose",
        #     self.amcl_pose_callback,
        #     10
        # )
        
        
        # Navigate to the goal
        # self.navigate(10.0, 10.0)
        # Follow the waypoints
        # self.follow_waypoints()

        
        self.get_logger().info("Navigation demo started")
        
    def localize(self):
        """
        Set the initial pose of the robot.
        """
        # Set the initial pose of the robot
        self._initial_pose.header.frame_id = "map"
        self._initial_pose.header.stamp = self._navigator.get_clock().now().to_msg()
        self._initial_pose.pose.position.x = 1.0
        self._initial_pose.pose.position.y = 1.0
        self._initial_pose.pose.position.z = 0.0
        self._initial_pose.pose.orientation.x = 0.0
        self._initial_pose.pose.orientation.y = 0.0
        self._initial_pose.pose.orientation.z = 0.0
        self._initial_pose.pose.orientation.w = 1.0
        self._navigator.setInitialPose(self._initial_pose)
        
    def navigate(self, x: float, y: float):
        """
        Navigate the robot to the goal (x, y).
        """
        self._navigator.waitUntilNav2Active() # Wait until Nav2 is active
        
        goal = self.create_pose_stamped(x, y, 0.0)
        
        self._navigator.goToPose(goal)
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")
        
        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")
            
    def follow_waypoints(self):
        self._navigator.waitUntilNav2Active()  # Wait until Nav2 is active

        pose1 = self.create_pose_stamped(-4.0, -3.0, 0.0)
        pose2 = self.create_pose_stamped(4.0, -4.0, 0.0)
        pose3 = self.create_pose_stamped(6.0, 4.0, 0.0)
        waypoints = [pose1, pose2, pose3]
        self._navigator.followWaypoints(waypoints)
        
        while not self._navigator.isTaskComplete():
            feedback = self._navigator.getFeedback()
            self.get_logger().info(f"Feedback: {feedback}")

        result = self._navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded")
        elif result == TaskResult.CANCELED:
            self.get_logger().info("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().info("Goal failed!")

    def create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """
        Create a PoseStamped message.
        """
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self._navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        self._initial_pose.pose.orientation.x = q_x
        self._initial_pose.pose.orientation.y = q_y
        self._initial_pose.pose.orientation.z = q_z
        self._initial_pose.pose.orientation.w = q_w

        goal.pose.orientation.x = q_x
        goal.pose.orientation.y = q_y
        goal.pose.orientation.z = q_z
        goal.pose.orientation.w = q_w
        return goal


def main(args=None):
    rclpy.init(args=args)
    node = NavigationDemoInterface("navigation_demo")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        rclpy.shutdown()