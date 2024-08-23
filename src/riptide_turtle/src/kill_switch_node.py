#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating a ROS 2 node
from std_msgs.msg import Bool  # Message type for publishing boolean values

# Define a class that extends the Node class
class TurtleEnablePublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'turtle_enable_publisher'
        super().__init__('turtle_enable_publisher')
        
        # Create a publisher that publishes Bool messages to the 'turtle_enabled' topic
        # The '10' specifies the queue size for the publisher
        self.publisher_ = self.create_publisher(Bool, 'turtle_enabled', 10)
        
        # Create a timer that calls the 'timer_callback' method every 30 seconds
        # The timer controls the rate at which messages are published
        self.timer = self.create_timer(30.0, self.timer_callback)
        
        # Initialize the state to False, meaning 'Disabled'
        # This will be used to alternate between Enabled and Disabled states
        self.state = False
        
        # Log a message indicating that the publisher node has started
        self.get_logger().info("Turtle Enable Publisher started!")

    # This method is called every 30 seconds by the timer
    def timer_callback(self):
        # Toggle the state: if it's currently False, set it to True, and vice versa
        self.state = not self.state
        
        # Create a new Bool message and set its data to the current state
        msg = Bool()
        msg.data = self.state
        
        # Publish the message to the 'turtle_enabled' topic
        self.publisher_.publish(msg)
        
        # Log the state that was just published, converting the boolean to a string
        state_str = "Enabled" if self.state else "Disabled"
        self.get_logger().info(f"Published: {state_str}")


# The main function that initializes and runs the ROS 2 node
def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the TurtleEnablePublisher node
    node = TurtleEnablePublisher()
    
    # Keep the node alive and processing callbacks (e.g., the timer callback)
    rclpy.spin(node)
    
    # Once the node is stopped, destroy it to free up resources
    node.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


# If this script is executed directly (rather than imported as a module), run the main function
if __name__ == '__main__':
    main()
