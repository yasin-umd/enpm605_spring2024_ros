from geometry_msgs.msg import PoseStamped
from navigation.robot_navigator_interface import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
import tf_transformations





def main(args=None):
    rclpy.init(args=args)
    leader_node = LeaderNavigationDemoInterface()
    follower_node = FollowerNavigationDemoInterface()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(leader_node)
    executor.add_node(follower_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        leader_node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        leader_node.destroy_node()
        follower_node.destroy_node()

        rclpy.shutdown()
