from typing import Tuple
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros
import math
from mage_msgs.msg import AdvancedLogicalCameraImage
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import PyKDL

# define different colors
class Color:
    """
    Define different colors.
    """

    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"  # Adding Cyan color
    MAGENTA = "\033[95m"  # Adding Magenta color
    END = "\033[0m"

def euler_from_quaternion(quaternion: Quaternion) -> Tuple[float, float, float]:
    """
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        quaternion (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: A tuple containing roll, pitch, and yaw
    """

    rotation = PyKDL.Rotation.Quaternion(
        quaternion.x, quaternion.y, quaternion.z, quaternion.w
    )
    return rotation.GetRPY()


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """
    Converts euler roll, pitch, yaw to a tuple of quaternion values (JPL convention).

    Returns:
        Quaternion: tuple of quaternion values (JPL convention)
    """
    quaternion = PyKDL.Rotation.RPY(roll, pitch, yaw).GetQuaternion()

    # Other way to do it
    # cy = math.cos(yaw * 0.5)
    # sy = math.sin(yaw * 0.5)
    # cp = math.cos(pitch * 0.5)
    # sp = math.sin(pitch * 0.5)
    # cr = math.cos(roll * 0.5)
    # sr = math.sin(roll * 0.5)

    # quaternion = [0] * 4
    # quaternion[0] = cy * cp * cr + sy * sp * sr
    # quaternion[1] = cy * cp * sr - sy * sp * cr
    # quaternion[2] = sy * cp * sr + cy * sp * cr
    # quaternion[3] = sy * cp * cr - cy * sp * sr

    return quaternion


def quaternion_to_euler(quaternion):
    euler = tf2_ros.transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )
    roll, pitch, yaw = euler

    output = "\n"
    output += "=" * 50 + "\n"
    output += f"Roll: : {math.degrees(roll)}\n"
    output += f"Pitch: : {math.degrees(pitch)}\n"
    output += f"Yaw: : {math.degrees(yaw)}\n"
    output += "=" * 50 + "\n"

    return roll, pitch, yaw


class BroadcasterListenerInterfaceDemo(Node):
    """
    Dynamic broadcaster and listener demo class.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        # Since we are using Gazebo, we need to set the use_sim_time parameter to True
        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        # -------------------
        # Broadcaster
        # -------------------
        # Parent frame
        self._part_parent_frame = "camera1_frame"
        # Child frame
        self._part_frame = "cube"
        # Create a dynamic broadcaster
        self._tf_dynamic_broadcaster = TransformBroadcaster(self)

        self._mutex_cbg1 = MutuallyExclusiveCallbackGroup()
        self._mutex_cbg2 = MutuallyExclusiveCallbackGroup()
        self._mutex_cbg3 = MutuallyExclusiveCallbackGroup()

        # subscriber to the topic /mage/advanced_logical_camera/image
        self._camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            "/mage/camera1/image",
            self.camera1_cb,
            qos_profile_sensor_data,
            callback_group=self._mutex_cbg1,
        )

        # Create a transform buffer and listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Listen to the transform between frames periodically
        self._listener_timer = self.create_timer(
            0.5, self._listener_cb, callback_group=self._mutex_cbg2
        )


        self.get_logger().info("Broadcaster/Listener demo started")


    def camera1_cb(self, msg: AdvancedLogicalCameraImage):
        """
        Callback function for the left_bins_camera subscriber.
        """

        # If no parts are detected, return
        if len(msg.part_poses) == 0:
            # self.get_logger().warn("No parts detected by the camera")
            return
        else:
            for part_pose in msg.part_poses:
                
                # position of the part
                x = part_pose.pose.position.x
                y = part_pose.pose.position.y
                z = part_pose.pose.position.z
                # orientation of the part
                orientation = part_pose.pose.orientation
                
                # green color
                
                output = "\n"
                output += "=" * 50 + "\n"
                output += "Part detected\n"
                output += f"Position: x: {x}, y: {y}, z: {z}\n"
                output += f"Orientation: x: {orientation.x}, y: {orientation.y}, z: {orientation.z}, w: {orientation.w}\n"
                output += "=" * 50 + "\n"
                self.get_logger().info(Color.YELLOW + output + Color.END)
                
                self.broadcast(
                    self._part_parent_frame, self._part_frame, part_pose.pose
                )

    def broadcast(self, parent, child, pose):
        """
        Build a transform message and broadcast it.

        Args:
            parent (str): Parent frame.
            child (str): Child frame.
            pose (geometry_msgs.msg.Pose): Pose of the child frame with respect to the parent frame.

        """
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent
        transform_stamped.child_frame_id = child

        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z
        transform_stamped.transform.rotation.x = pose.orientation.x
        transform_stamped.transform.rotation.y = pose.orientation.y
        transform_stamped.transform.rotation.z = pose.orientation.z
        transform_stamped.transform.rotation.w = pose.orientation.w

        output = "\n"
        output += "=" * 50 + "\n"
        output += f"Broadcasting transform between {parent} and {child}\n"
        output += f"Translation: x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}\n"
        output += f"Rotation: x: {pose.orientation.x}, y: {pose.orientation.y}, z: {pose.orientation.z}, w: {pose.orientation.w}\n"
        output += "=" * 50 + "\n"
        self.get_logger().info(Color.CYAN + output + Color.END)
        

        self._tf_dynamic_broadcaster.sendTransform(transform_stamped)

    def _listener_cb(self):
        """
        Callback function for the listener timer.
        """

        try:
            # Get the transform between frames
            transform = self._tf_buffer.lookup_transform(
                "world", self._part_frame, rclpy.time.Time()
            )

            euler = euler_from_quaternion(transform.transform.rotation)

            output = "\n"
            output += "=" * 50 + "\n"
            output += f"Transform from {self._part_frame} to world \n"
            output += f"Translation: x: {transform.transform.translation.x}, y: {transform.transform.translation.y}, z: {transform.transform.translation.z}\n"
            output += f"Rotation: rpy: {euler[0]}, {euler[1]}, {euler[2]}\n"
            output += "=" * 50 + "\n"
            self.get_logger().info(Color.GREEN + output + Color.END)


        except TransformException as ex:
            self.get_logger().fatal(
                f"Could not get transform between world and {self._part_frame}: {str(ex)}"
            )


def broadcaster_listener_main(args=None):
    rclpy.init(args=args)
    node = BroadcasterListenerInterfaceDemo("broadcaster_listener_demo")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        rclpy.shutdown()
