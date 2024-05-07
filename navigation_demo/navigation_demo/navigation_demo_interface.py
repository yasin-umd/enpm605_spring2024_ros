from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations


class NavigationDemoInterface(Node):
    """
    Example of a class that uses the BasicNavigator class to navigate the robot.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        # Since we are using Gazebo, we need to set the use_sim_time parameter to True
        self._sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([self._sim_time])
        
        # Navigator
        self._navigator = BasicNavigator()
        # Initial pose
        self._initial_pose = PoseStamped()

        # Set the initial pose of the robot
        self.localize()
        # Navigate to the goal
        self.navigate(10.0, 10.0)

        
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
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        self._initial_pose.pose.orientation.x = q_x
        self._initial_pose.pose.orientation.y = q_y
        self._initial_pose.pose.orientation.z = q_z
        self._initial_pose.pose.orientation.w = q_w

        goal.pose.orientation.x = q_x
        goal.pose.orientation.y = q_y
        goal.pose.orientation.z = q_z
        goal.pose.orientation.w = q_w
        
        
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