#!/usr/bin/env python3

import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from rclpy
from geometry_msgs.msg import Twist  # Import the Twist message type for controlling the turtle

class TurtleNode(Node): # Define a new class 'TurtleNode' that inherits from the ROS 2 'Node' class
    def __init__(self): # The constructor method to initialize the node's attributes and setup
        super().__init__('turtle_node')  # Initialize the node with the name 'turtle_node'
        
        # TODO: Create a publisher that publishes Twist messages to the '/turtle1/cmd_vel' topic
        # self.publisher_ = ...

        # TODO: Set up a timer that periodically calls the publish_twist method
        # self.timer = ...

        self.get_logger().info("Turtle Node started!")  # Log a message to confirm the node has started

    def publish_twist(self):
        # Create and configure a Twist message to move the turtle
        twist = Twist()
        twist.linear.x = 2.0  # Set linear velocity in the x direction
        twist.angular.z = 1.0  # Set angular velocity around the z-axis (yaw)

        # TODO: Publish the Twist message
        # self.publisher_.publish(msg)

        self.get_logger().info("Published twist message")  # Log a message after publishing the twist

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = TurtleNode()  # Create an instance of the TurtleNode
    rclpy.spin(node)  # Keep the node running until manually interrupted
    node.destroy_node()  # Clean up the node after shutdown
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  # Run the main function if this script is executed directly