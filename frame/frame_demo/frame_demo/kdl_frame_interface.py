import rclpy
import PyKDL
from typing import Tuple
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from mage_msgs.msg import AdvancedLogicalCameraImage
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter


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


class KDLFrameDemo(Node):
    """
    Class to broadcast Frames. This class consists of a static broadcaster and a dynamic broadcaster.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])
        # subscriber to the topic /mage/advanced_logical_camera/image
        self._camera_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            "/mage/camera1/image",
            self.camera1_cb,
            qos_profile_sensor_data,
        )

        # Name of the frame for camera1
        self._camera1_frame = "camera1_frame"

        self.get_logger().info("KDL frame demo started")

    def camera1_cb(self, msg: AdvancedLogicalCameraImage):
        """
        Callback function for the left_bins_camera subscriber.
        """
        part_pose_in_camera = None
        camera_pose_in_world = None

        # If no parts are detected, return
        if len(msg.part_poses) == 0:
            return
        else:
            camera_pose_in_world = msg.sensor_pose
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
                part_pose_in_camera = part_pose.pose

                # Compute the pose of the part in world frame
                self.compute_part_pose_in_world(
                    part_pose_in_camera, camera_pose_in_world
                )

    def compute_part_pose_in_world(
        self, part_pose_in_camera_frame, camera_pose_in_world_frame
    ):
        # First frame
        camera_orientation = camera_pose_in_world_frame.orientation
        camera_x = camera_pose_in_world_frame.position.x
        camera_y = camera_pose_in_world_frame.position.y
        camera_z = camera_pose_in_world_frame.position.z

        kdl_frame_camera_world = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ),
            PyKDL.Vector(camera_x, camera_y, camera_z),
        )

        # Second frame
        part_orientation = part_pose_in_camera_frame.orientation
        part_x = part_pose_in_camera_frame.position.x
        part_y = part_pose_in_camera_frame.position.y
        part_z = part_pose_in_camera_frame.position.z

        kdl_frame_part_camera = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                part_orientation.x,
                part_orientation.y,
                part_orientation.z,
                part_orientation.w,
            ),
            PyKDL.Vector(part_x, part_y, part_z),
        )

        # Multiply the two frames
        kdl_frame_part_world = kdl_frame_camera_world * kdl_frame_part_camera

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = kdl_frame_part_world.p.x()
        pose.position.y = kdl_frame_part_world.p.y()
        pose.position.z = kdl_frame_part_world.p.z()

        q = kdl_frame_part_world.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # Print the pose
        rpy = euler_from_quaternion(pose.orientation)
        output = "\n"
        output += "=" * 50 + "\n"
        output += f"Part position in world frame: \n x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}\n"
        output += (
            f"Part orientation in world frame: \n rpy: {rpy[0]}, {rpy[1]}, {rpy[2]}\n"
        )
        output += "=" * 50 + "\n"
        self.get_logger().info(Color.GREEN + output + Color.END)


def kdlframe_main(args=None):
    rclpy.init(args=args)
    node = KDLFrameDemo("kdl_frame_demo")
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        rclpy.shutdown()
