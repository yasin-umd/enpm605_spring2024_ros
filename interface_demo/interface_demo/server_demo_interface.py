import rclpy
from rclpy.node import Node
from interface_demo_msgs.srv import RobotRotate
from geometry_msgs.msg import Twist
from math import radians
from rclpy.executors import MultiThreadedExecutor



class ServerDemoInterface(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Create the service
        self._robot_rotate_srv = self.create_service(
            RobotRotate, "robot_rotate", self.robot_command_cb
        )
        self.get_logger().info("robot_command service has been initialized")
        # Create a publisher to send Twist messages to the robot
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        
        
    def robot_command_cb(self, request, response):
        self.get_logger().info("Request received")
        # Get rotatation direction
        if request.direction == RobotRotate.Request.CCW:
            angular_velocity = request.angular_velocity
        elif request.direction == RobotRotate.Request.CW:
            angular_velocity = -request.angular_velocity
        else:
            response.success = False
            response.message = "Invalid direction"
            return response
        
        # Convert angle from degrees to radians
        angle_rad = radians(request.angle)
        # Calculate duration based on the angle and velocity
        duration = abs(angle_rad / request.angular_velocity)
        # Current time (s)
        # extract the seconds from the time
        current_time = self.get_clock().now().to_msg().sec
        # End time (s)
        end_time = current_time + duration
        
        rate = self.create_rate(10)
        while self.get_clock().now().to_msg().sec < end_time:
            twist = Twist()
            twist.angular.z = angular_velocity
            self._cmd_vel_pub.publish(twist)
            rate.sleep()

        # Stop the robot
        twist = Twist()
        self._cmd_vel_pub.publish(twist)

        response.success = True
        response.message = f"Successfully rotated robot {request.angle} degrees at velocity {request.angular_velocity}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServerDemoInterface("server_demo")
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
