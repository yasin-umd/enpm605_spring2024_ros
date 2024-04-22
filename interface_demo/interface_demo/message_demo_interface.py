import rclpy
import os
from rclpy.node import Node
from interface_demo_msgs.msg import RobotStatus
from interface_demo_msgs.srv import RobotStartStop
from geometry_msgs.msg import Twist

class MessageDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Get the value for the environment variable TURTLEBOT3_MODEL
        model = os.getenv("TURTLEBOT3_MODEL")
        # check if the environment variable is set
        if model is None:
            self.get_logger().error("Environment variable TURTLEBOT3_MODEL is not set")
            return
        self._model = MessageDemoInterface.convert_model_to_int(os.getenv("TURTLEBOT3_MODEL"))
        
        # Variable to hold the current linear velocity of the robot
        self._current_linear_velocity = 0.0
        # Variable to hold the total distance traveled
        self._distance_traveled = 0.0
        # The time at the last velocity command, initialized to None
        self._last_cmd_vel_time = None
        # Initialize the message to publish robot status
        self._robot_status_msg = RobotStatus()
        # Initialize the message to publish velocity command
        self._cmd_vel_msg = Twist()

        # Create the service
        self._start_stop_srv = self.create_service(
            RobotStartStop, "robot_start_stop", self.robot_start_stop_cb
        )
        self._stop = False
        
        self._sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.sub_cmd_vel_cb, 10)
        self._pub_robot_status = self.create_publisher(RobotStatus, "robot_status", 10)
        self._pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self._timer_cmd_vel = self.create_timer(0.5, self.timer_cmd_vel_cb)
        self._timer_robot_status = self.create_timer(0.2, self.timer_robot_status_cb)
    
    @staticmethod
    def convert_model_to_int(model):
        """
        Helper function to convert the model name to an integer value

        Args:
            model (str): The model name of the robot

        Returns:
            int: The integer value corresponding to the model name
        """
        if model == "burger":
            return RobotStatus.BURGER
        elif model == "waffle":
            return RobotStatus.WAFFLE
        elif model == "waffle_pi":
            return RobotStatus.WAFFLE_PI
    
    
        
    def sub_cmd_vel_cb(self, msg):
        self._current_linear_velocity = msg.linear.x
        
    def timer_cmd_vel_cb(self):
        if self._stop:
            self._cmd_vel_msg.linear.x = 0.0
            self._pub_cmd_vel.publish(self._cmd_vel_msg)
            return
        self._cmd_vel_msg.linear.x = 0.1
        self._pub_cmd_vel.publish(self._cmd_vel_msg)

    def timer_robot_status_cb(self):
        # compute distance traveled by robot using current linear velocity
        current_time = self.get_clock().now()
        
        if self._last_cmd_vel_time is not None:
            time_diff = current_time - self._last_cmd_vel_time
            time_diff_sec = time_diff.nanoseconds / 1e9
            distance = self._current_linear_velocity * time_diff_sec
            self._distance_traveled += distance
        
        self._robot_status_msg.distance = self._distance_traveled
        self._robot_status_msg.model = self._model
        self._pub_robot_status.publish(self._robot_status_msg)
        
        self._last_cmd_vel_time = current_time

    def robot_start_stop_cb(self, request, response):
        if request.action == RobotStartStop.Request.START:
            self._stop = False
            response.message = "Robot started"
        elif request.action == RobotStartStop.Request.STOP:
            self._stop = True
            response.message = "Robot stopped"
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MessageDemoInterface("message_demo")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        node.destroy_node()
        # clean up the node
        rclpy.shutdown()