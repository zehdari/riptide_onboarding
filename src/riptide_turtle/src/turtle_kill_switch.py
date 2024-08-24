#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class TurtleEnablePublisher(Node):
    def __init__(self):
        super().__init__('turtle_kill_switch_publisher')
        self.publisher_ = self.create_publisher(Bool, 'turtle_enabled', 10)
        self.state_timer = self.create_timer(30.0, self.toggle_state_callback)
        self.publish_timer = self.create_timer(1.0, self.publish_state_callback)
        self.state = True
        self.get_logger().info("Turtle Enable Publisher started!")

    def toggle_state_callback(self):
        self.state = not self.state
        state_str = "Enabled" if self.state else "Disabled"
        self.get_logger().info(f"State toggled: {state_str}")

    def publish_state_callback(self):
        msg = Bool()
        msg.data = self.state
        self.publisher_.publish(msg)
        state_str = "Enabled" if self.state else "Disabled"
        self.get_logger().info(f"Published: {state_str}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleEnablePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
