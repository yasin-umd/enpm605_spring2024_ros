#!/usr/bin/env python3

import rclpy
import time

from final_project.environment_startup import EnvironmentStartup

def main():
    rclpy.init()

    startup_node = EnvironmentStartup()

    # Wait 2 seconds for gazebo to start up
    time.sleep(2)

    # Spawn sensors
    startup_node.spawn_sensors()
    try:
        rclpy.spin(startup_node)
    except KeyboardInterrupt:
        startup_node.get_logger().info("Keyboard Interrupt (SIGINT) detected")
    finally:
        startup_node.destroy_node()
        # clean up the node
        rclpy.shutdown()


if __name__ == '__main__':
    main()