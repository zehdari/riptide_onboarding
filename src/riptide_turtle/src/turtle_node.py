#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating a ROS 2 node
from geometry_msgs.msg import Twist  # Message type for velocity commands (linear and angular)

class TurtleNode(Node):
    # Node class that publishes Twist messages to control the turtle in turtlesim
    def __init__(self):
        super().__init__('turtle_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_twist)
        self.get_logger().info("Turtle Node started!")
    
    def publish_twist(self):
        # Publishes a hardcoded Twist message to the '/turtle1/cmd_vel' topic
        twist = Twist()
        twist.linear.x = 2.0
        twist.angular.z = 1.0
        self.publisher_.publish(twist)
        self.get_logger().info("Published twist message")

def main(args=None):
    # Main function to initialize the node and keep it spinning
    rclpy.init(args=args)
    node = TurtleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
