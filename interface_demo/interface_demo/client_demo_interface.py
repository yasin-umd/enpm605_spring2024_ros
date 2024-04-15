import rclpy
from rclpy.node import Node
from interface_demo_msgs.srv import RobotRotate, RobotStartStop
from interface_demo_msgs.msg import RobotStatus
from rclpy.executors import MultiThreadedExecutor


class ClientDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Create a service client to send the robot start/stop command
        self._start_stop_client = self.create_client(RobotStartStop, "robot_start_stop")
        while not self._start_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'robot_start_stop' is not available, waiting...")

        # Create a service client to send the robot rotate command
        self._robot_rotate_client = self.create_client(RobotRotate, "robot_rotate")
        while not self._start_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service 'robot_rotate' is not available, waiting...")

        # Subscriber to receive the robot status
        self._sub_robot_status = self.create_subscription(
            RobotStatus, "robot_status", self.sub_robot_status_cb, 10
        )
        
        self._robot_stopped = False
        self._robot_rotated = False
        self._robot_moved = False
        self._stop_requested = False
        self._rotate_requested = False
        self._move_requested = False

    def sub_robot_status_cb(self, msg: RobotStatus):
        """
        Callback function to receive the robot status.
        
        This version send requests directly from the callback function.
        The requests are sent asynchonously, so the callback function will not block.

        Args:
            msg (RobotStatus): The message containing the robot status
        """
        
        if self._robot_moved:
            self.destroy_subscription(self._sub_robot_status)
        
        self.get_logger().info(f"Received distance: {msg.distance}")
        
        # stop the robot
        if not self._stop_requested:
            if msg.distance >= 2.0 and not self._robot_stopped:
                # stop the robot
                self.get_logger().info("Stopping the robot")
                self._stop_requested = True
                self._start_stop_robot(RobotStartStop.Request.STOP)
        
        # rotate the robot
        if not self._rotate_requested:
            if self._robot_stopped and not self._robot_rotated:
                self.get_logger().info("Rotating the robot")
                self._rotate_requested = True
                self._rotate_robot()

        # move the robot
        if not self._move_requested:
            if self._robot_rotated:
                self.get_logger().info("Moving the robot")
                self._move_requested = True
                self._move_robot()

    def _move_robot(self):
        """
        Helper function to send the move command to the robot.
        """
        request = RobotStartStop.Request()
        request.action = RobotStartStop.Request.START
        future = self._start_stop_client.call_async(request)
        future.add_done_callback(self.start_stop_cb)
        
    def _rotate_robot(self):
        """
        Helper function to send the rotate command to the robot.
        """
        request = RobotRotate.Request()
        request.angle = 90.0
        request.angular_velocity = 0.3
        request.direction = RobotRotate.Request.CCW
        future = self._robot_rotate_client.call_async(request)
        future.add_done_callback(self.rotate_cb)
        
        
    def _start_stop_robot(self, action):
        """
        Helper function to send the start/stop command to the robot.

        Args:
            action (int): The command to send to the robot
        """
        request = RobotStartStop.Request()
        request.action = action
        future = self._start_stop_client.call_async(request)
        # Add a callback function to be called when the future is complete
        future.add_done_callback(self.start_stop_cb)

    def start_stop_cb(self, future):
        """
        Callback function to process the response from the start/stop service.

        Args:
            future (Future): The future object to get the response from
        """
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response.message}")
            self._robot_stopped = response.success
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
            
    def rotate_cb(self, future):
        """
        Callback function to process the response from the rotate service.

        Args:
            future (Future): The future object to get the response from
        """
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response.message}")
            self._robot_rotated = response.success
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
            
    def move_cb(self, future):
        """
        Callback function to process the response from the move service.

        Args:
            future (Future): The future object to get the response from
        """
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response.message}")
            self._robot_moved = response.success
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ClientDemoInterface("client_demo")
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
