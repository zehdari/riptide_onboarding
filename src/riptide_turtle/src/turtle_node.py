#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from example_interfaces.srv import Trigger

class TurtleNode(Node):
    def __init__(self):
        super().__init__('turtle_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_twist)

        self.declare_parameter('linear_speed', 2.0)
        self.declare_parameter('angular_speed', 1.0)

        self.reset_speed_service = self.create_service(Trigger, 'reset_speed', self.reset_speed_callback)

        self.enabled = False
        self.get_logger().info("Turtle Node started!")

    def enabled_callback(self, msg):
        self.enabled = msg.data
        state_str = "Enabled" if self.enabled else "Disabled"
        self.get_logger().info(f"Received state: {state_str}")

    def publish_twist(self):
        if self.enabled:
            linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
            angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.publisher_.publish(twist)
            self.get_logger().info(f"Published twist message with linear_speed={linear_speed}, angular_speed={angular_speed}")

    def reset_speed_callback(self, request, response):
        self.set_parameters([
            rclpy.parameter.Parameter('linear_speed', rclpy.Parameter.Type.DOUBLE, 2.0),
            rclpy.parameter.Parameter('angular_speed', rclpy.Parameter.Type.DOUBLE, 1.0)
        ])
        self.get_logger().info("Linear and Angular speeds reset to default values")
        response.success = True
        response.message = "Speeds have been reset to default values"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
