#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # Initial logging messages
        self.get_logger().info("Hello from test node")
        self.get_logger().warn("This is a warning")
        self.get_logger().error("This is an error")

        # Log that the node has started
        self.get_logger().info("Test node has been started.")

        # Create a timer that calls the timer_callback function at a specified rate
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        # Log a message every time the timer fires
        self.get_logger().info("Hello")

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)

    # Clean up and shutdown
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
