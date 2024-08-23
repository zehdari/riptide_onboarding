#!/usr/bin/env python3

import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating a ROS 2 node
from geometry_msgs.msg import Twist  # Message type for velocity commands (linear and angular)
from std_msgs.msg import Bool  # Message type for boolean values
from rclpy.task import Future

class TurtleNode(Node):
    def __init__(self):
        # Initialize the node with the name 'turtle_node'
        super().__init__('turtle_node')
        
        # Create a publisher that publishes Twist messages to the 'turtle_twist' topic
        self.publisher_ = self.create_publisher(Twist, 'turtle_twist', 10)
        
        # Create a client for the 'spawn' service to spawn a turtle
        self.spawn_client = self.create_client(
            rclpy.types.ServiceType("turtlesim/srv/Spawn"), '/spawn'
        )
        
        # Create a client for the 'kill' service to kill a turtle
        self.kill_client = self.create_client(
            rclpy.types.ServiceType("turtlesim/srv/Kill"), '/kill'
        )
        
        # Create a subscriber that listens to the 'turtle_enabled' topic
        self.subscription = self.create_subscription(
            Bool, 
            'turtle_enabled', 
            self.turtle_enabled_callback, 
            10
        )
        
        # Initialize the state to determine if the turtle is enabled
        self.turtle_enabled = False
        
        # Log a message indicating that the turtle node has started
        self.get_logger().info("Turtle Node started!")
    
    # Callback function for when the turtle_enabled topic is updated
    def turtle_enabled_callback(self, msg):
        self.turtle_enabled = msg.data
        if self.turtle_enabled:
            self.get_logger().info("Turtle is enabled, publishing twist.")
            self.publish_twist()
        else:
            self.get_logger().info("Turtle is disabled, no twist published.")
    
    # Function to publish a hardcoded twist to control the turtle
    def publish_twist(self):
        # Create a new Twist message
        twist = Twist()
        twist.linear.x = 2.0  # Move forward
        twist.angular.z = 1.0  # Turn with a constant angular velocity
        
        # Publish the twist message
        self.publisher_.publish(twist)
    
    # Function to call the turtle_spawn service to spawn a turtle at specific coordinates
    def call_spawn_service(self, x, y, theta, name='turtle1'):
        # Wait for the spawn service to be available
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        
        # Create a request to spawn the turtle
        request = rclpy.types.ServiceType.RequestType("turtlesim/srv/Spawn")
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        # Call the service and wait for the result
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Turtle spawned with name: {future.result().name}")
        else:
            self.get_logger().error("Failed to spawn the turtle.")
    
    # Function to call the turtle_kill service to remove the turtle
    def call_kill_service(self, name='turtle1'):
        # Wait for the kill service to be available
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill service not available, waiting...')
        
        # Create a request to kill the turtle
        request = rclpy.types.ServiceType.RequestType("turtlesim/srv/Kill")
        request.name = name
        
        # Call the service and wait for the result
        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"Turtle '{name}' killed successfully.")
        else:
            self.get_logger().error(f"Failed to kill the turtle '{name}'.")
    

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the TurtleNode
    node = TurtleNode()
    
    # Example of spawning a turtle at specific coordinates
    node.call_spawn_service(5.0, 5.0, 0.0, 'turtle1')
    
    # Keep the node alive and processing callbacks
    rclpy.spin(node)
    
    # Example of killing the turtle
    node.call_kill_service('turtle1')
    
    # Destroy the node when done
    node.destroy_node()
    
    # Shutdown the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
