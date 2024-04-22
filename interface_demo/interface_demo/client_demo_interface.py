import rclpy
from rclpy.node import Node
from interface_demo_msgs.srv import RobotRotate, RobotStartStop
from interface_demo_msgs.msg import RobotStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class ClientDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Create a service client to start and stop the robot
        # The service will be called synchronously
        # A callback group is needed to avoid deadlocks
        self._mutex_cbg = MutuallyExclusiveCallbackGroup()
        self._start_stop_client = self.create_client(
            srv_type=RobotStartStop, 
            srv_name="robot_start_stop", 
            callback_group=self._mutex_cbg
        )

        # Create a service client to send requests to rotate the robot
        # The service will be called asynchronously
        # A callback group is not needed but a callback function is needed
        # to process the response
        self._robot_rotate_client = self.create_client(RobotRotate, "robot_rotate")

        # Create a timer to send commands to the robot
        self._timer = self.create_timer(0.5, self.timer_cb)

        # Subscriber to receive the robot status
        self._sub_robot_status = self.create_subscription(
            RobotStatus, "robot_status", self.sub_robot_status_cb, 10
        )

        # Flags for state-based decision making
        self._stop_requested = False
        self._rotate_requested = False
        self._move_requested = False

    def timer_cb(self):
        """
        Timer callback function to send commands to the robot.
        """
        # rotate the robot
        if self._rotate_requested:
            self.get_logger().info("Rotating the robot")
            # Prevent the robot from rotating again
            self._rotate_requested = False
            # Send the rotate command
            self._rotate_robot()
        if self._move_requested:
            self.get_logger().info("Moving the robot")
            # Prevent the robot from moving again
            self._move_requested = False
            self._start_stop_robot(RobotStartStop.Request.START)

    def sub_robot_status_cb(self, msg: RobotStatus):
        """
        Callback function to receive the robot status.
        The callback processes messages until the robot has stopped.

        Args:
            msg (RobotStatus): The message containing the robot status
        """

        # No need to process the message if the robot has already stopped
        if self._stop_requested:
            return

        self.get_logger().info(f"Traveled distance: {msg.distance}")

        # stop the robot
        if not self._stop_requested:
            if msg.distance >= 2.0:
                # stop the robot
                self.get_logger().info("Stopping the robot")
                self._start_stop_robot(RobotStartStop.Request.STOP)

    def _rotate_robot(self):
        """
        Build and send the request to rotate the robot.
        """
        # Wait until the service is available, the timeout is 0.5 seconds
        while not self._robot_rotate_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(
                "Service 'robot_rotate' is not available, waiting..."
            )
            
        request = RobotRotate.Request()
        request.angle = 90.0
        request.angular_velocity = 0.3
        request.direction = RobotRotate.Request.CCW
        future = self._robot_rotate_client.call_async(request)
        future.add_done_callback(self.rotate_robot_cb)

    def rotate_robot_cb(self, future):
        """
        Callback function to process the response from the 'robot_rotate' service.

        Args:
            future (Future): The future object to get the response from
        """
        try:
            response = future.result()
            self.get_logger().info(f"Response: {response.message}")
            if response.success:
                self._move_requested = True
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def _start_stop_robot(self, action):
        """
        Build and send the request to start or stop the robot

        Args:
            action (int): The command to send to the robot
        """
        if action == RobotStartStop.Request.START:
            self.get_logger().info("Starting the robot")
        elif action == RobotStartStop.Request.STOP:
            self.get_logger().info("Stopping the robot")
            self._stop_requested = True

        # Wait until the service is available, the timeout is 0.5 seconds
        while not self._start_stop_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(
                "Service 'robot_start_stop' is not available, waiting..."
            )
            
        request = RobotStartStop.Request()
        request.action = action
        # Synchronous call
        future = self._start_stop_client.call(request)
        try:
            self.get_logger().info(f"Response message: {future.message}")
            self.get_logger().info(f"Response success: {future.success}")
            if future.success:
                if action == RobotStartStop.Request.START:
                    self._move_requested = True
                elif action == RobotStartStop.Request.STOP:
                    self._rotate_requested = True
        except Exception as e:
            self.get_logger().error(f"Failed to execute service call: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ClientDemoInterface("client_demo")
    # MultiThreadedExecutor is used since we are using callback groups
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
