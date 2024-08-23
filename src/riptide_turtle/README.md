# TurtleNode Tutorial - ROS 2

This tutorial guides you through creating and enhancing a ROS 2 node (`TurtleNode`) that interacts with the turtlesim simulation. The tutorial is divided into several steps, each building upon the last to add more functionality and interactivity.

---

## Step 1: Create a Basic TurtleNode

### Objective

Create a ROS 2 node that continuously publishes `Twist` messages to control the turtle in the turtlesim simulation.

### Implementation

1. **Create a new Python script (`turtle_node.py`) in your ROS 2 package:**

   ```python
    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    class TurtleNode(Node):
        def __init__(self):
            super().__init__('turtle_node')
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.timer = self.create_timer(1.0, self.publish_twist)
            self.get_logger().info("Turtle Node started!")

        def publish_twist(self):
            twist = Twist()
            twist.linear.x = 2.0
            twist.angular.z = 1.0
            self.publisher_.publish(twist)
            self.get_logger().info("Published twist message")

    def main(args=None):
        rclpy.init(args=args)
        node = TurtleNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

   ```

2. **Build the Package:**

   Before running the node, ensure you are in the osu-uwrt/development/software directory and build the riptide_turtle package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

   **Note:** You will need to do this everytime you update a node file.

3. **Run the node:**

   ```bash
   ros2 run riptide_onboarding turtle_node.py
   ```

### Result

The turtle should move in a circle continuously as the node publishes `Twist` messages with a fixed linear and angular velocity.

---

## Step 2: Create a Kill Switch Node

### Objective

Create a separate ROS 2 node that publishes to the `/turtle_enabled` topic, enabling or disabling the turtle's movement.

### Implementation

1. **Create a new Python script (`turtle_kill_switch.py`) in your ROS 2 package:**

   ```python
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

   ```

2. **Rebuild the Package:**

   Before running the node, ensure you are in the osu-uwrt/development/software directory and rebuild the riptide_turtle package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

3. **Run the Kill Switch Node:**

   ```bash
   ros2 run riptide_onboarding turtle_kill_switch.py
   ```

4. **Check if the Kill Switch is Publishing:**

   You can verify that the `turtle_kill_switch` node is publishing messages to the `/turtle_enabled` topic by using the following command:

   ```bash
   ros2 topic echo /turtle_enabled
   ```

   You should see alternating `True` and `False` values being published every 30 seconds.

### Result

The kill switch node alternates the state of the turtle between enabled and disabled every 30 seconds and publishes this state every second.

---

## Step 3: Integrate the Kill Switch with the TurtleNode

### Objective

Modify the `TurtleNode` to listen to the `/turtle_enabled` topic and only publish `Twist` messages when the turtle is enabled.

### Implementation

1. **Update the `TurtleNode` to include a subscriber:**

   ```python
    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool

    class TurtleNode(Node):
        def __init__(self):
            super().__init__('turtle_node')
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)
            self.timer = self.create_timer(1.0, self.publish_twist)
            self.enabled = False
            self.get_logger().info("Turtle Node started!")

        def enabled_callback(self, msg):
            self.enabled = msg.data
            state_str = "Enabled" if self.enabled else "Disabled"
            self.get_logger().info(f"Received state: {state_str}")

        def publish_twist(self):
            if self.enabled:
                twist = Twist()
                twist.linear.x = 2.0
                twist.angular.z = 1.0
                self.publisher_.publish(twist)
                self.get_logger().info("Published twist message")

    def main(args=None):
        rclpy.init(args=args)
        node = TurtleNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

   ```

2. **Rebuild the Package:**

   Before running the nodes, ensure you are in the osu-uwrt/development/software directory and again rebuild the riptide_turtle package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

3. **Run the nodes:**

   Start both the `TurtleNode` and `turtle_kill_switch` nodes:

   ```bash
   ros2 run riptide_onboarding turtle_node.py
   ros2 run riptide_onboarding turtle_kill_switch.py
   ```

### Result

The turtle will only move when the `/turtle_enabled` topic publishes `True`, as controlled by the `turtle_kill_switch` node.

---

## Step 4: Add Dynamic Parameters

### Objective

Introduce ROS 2 parameters to dynamically adjust the turtle's linear and angular speeds.

### Implementation

1. **Update the `TurtleNode` to use parameters for speed control:**

   ```python
    #!/usr/bin/env python3

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool

    class TurtleNode(Node):
        def __init__(self):
            super().__init__('turtle_node')
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
            self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)
            self.timer = self.create_timer(1.0, self.publish_twist)

            self.declare_parameter('linear_speed', 2.0)
            self.declare_parameter('angular_speed', 1.0)

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

    def main(args=None):
        rclpy.init(args=args)
        node = TurtleNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

   ```

2. **Rebuild the Package:**

   Before running the node, ensure you are in the osu-uwrt/development/software directory and rebuild the riptide_turtle package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

3. **Restart the turtle node:**

   If the turtle node is currently running, ctrl+c in the terminal to stop it. Then, restart the node:

   ```bash
   ros2 run riptide_onboarding turtle_node.py
   ```

4. **Change the parameters dynamically:**

   ```bash
   ros2 param set /turtle_node linear_speed 3.0
   ros2 param set /turtle_node angular_speed 2.0
   ```

### Result

The turtle’s speed changes dynamically based on the parameter values.

---

## Step 5: Add a Service to Reset Speeds

### Objective

Add a service that resets the linear and angular speeds to their original values.

### Implementation

1. **Update the `TurtleNode` to include a reset speed service:**

   ```python
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

   ```

2. **Rebuild the Package:**

   Before running the node, ensure you are in the osu-uwrt/development/software directory and rebuild the riptide_turtle package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

3. **Restart the turtle node:**

   If the turtle node is currently running, ctrl+c in the terminal to stop it. Then, restart the node:

   ```bash
   ros2 run riptide_onboarding turtle_node.py
   ```

4. **Call the service to reset speeds:**

   ```bash
   ros2 service call /reset_speed example_interfaces/srv/Trigger
   ```

### Result

The turtle’s linear and angular speeds are reset to the original values.

---

### Conclusion

Throughout these steps, you've built a flexible and interactive ROS 2 node that controls a turtle in the turtlesim simulation. You've learned how to:

- Continuously publish movement commands
- Control the node's behavior with topics
- Dynamically adjust parameters at runtime
- Implement and use a service to reset parameters

These concepts form a solid foundation for developing more advanced ROS 2 applications. Keep experimenting with new features and functionalities!
