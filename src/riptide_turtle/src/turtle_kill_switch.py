#!/usr/bin/env python3

import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from rclpy
from std_msgs.msg import Bool  # Import the Bool message type for enabling/disabling the turtle

class TurtleEnablePublisher(Node):  # Define a new class 'TurtleEnablePublisher' that inherits from the ROS 2 'Node' class
    def __init__(self):  # The constructor method to initialize the node's attributes and setup
        super().__init__('turtle_kill_switch_publisher')  # Initialize the node with the name 'turtle_kill_switch_publisher'
        
        # TODO: Create a publisher that publishes Bool messages to the 'turtle_enabled' topic
        # self.publisher_ = ...

        # TODO: Set up a timer that periodically calls the toggle_state_callback method every 30 seconds
        # self.state_timer = ...

        # TODO: Set up a timer that periodically calls the publish_state_callback method every second
        # self.publish_timer = ...

        self.state = True  # Initialize the state as True (enabled)
        self.get_logger().info("Turtle Enable Publisher started!")  # Log a message to confirm the node has started

    def toggle_state_callback(self):
        # Toggle the state between True and False
        self.state = not self.state
        state_str = "Enabled" if self.state else "Disabled"
        self.get_logger().info(f"State toggled: {state_str}")

    def publish_state_callback(self):
        # Create and configure a Bool message with the current state
        msg = Bool()
        msg.data = self.state

        # TODO: Publish the Bool message
        # self.publisher_.publish(msg)

        state_str = "Enabled" if self.state else "Disabled"
        self.get_logger().info(f"Published: {state_str}")

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = TurtleEnablePublisher()  # Create an instance of the TurtleEnablePublisher
    rclpy.spin(node)  # Keep the node running until manually interrupted
    node.destroy_node()  # Clean up the node after shutdown
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  # Run the main function if this script is executed directly