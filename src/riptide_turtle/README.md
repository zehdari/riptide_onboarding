# TurtleNode Tutorial - ROS 2

This tutorial guides you through creating and enhancing a ROS 2 node (`TurtleNode`) that interacts with the turtlesim simulation. The tutorial is divided into several steps, each building upon the last to add more functionality and interactivity.

---

## Step 1: Create a Basic TurtleNode

**Objective:**

Create a ROS 2 node that continuously publishes `Twist` messages to control the turtle in the turtlesim simulation.

**Guided Implementation:**

1. **Create a new Python script (`turtle_node.py`) in your ROS 2 package.**

   Below is the skeleton of your `TurtleNode` class, including the `main()` function and the `Twist` message.  
   - The skeleton is located in `src/turtle_node.py`  
   - The topic you will need to publish to is `/turtle1/cmd_vel`  
  
   Your task is to implement the missing functionality:

   ```python
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
   ```
   
    **Helpful Links:**

    - [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

    **Hints:**

    <details>
    <summary>Hint 1 - Creating the Publisher</summary>

    To publish `Twist` messages that control the turtle's movement, you need to create a publisher. Here’s how you can do that:

    ```python
    self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
    ```

    - `Twist`: The message type used to control the turtle's movement.
    - `/turtle1/cmd_vel`: The topic where movement commands are sent.
    - `10`: The queue size, which controls how many messages can be buffered.

    </details>

    <details>
    <summary>Hint 2 - Setting Up the Timer</summary>

    You need to periodically call the `publish_twist` function to send movement commands. Here’s how to set up a timer:

    ```python
    self.timer = self.create_timer(1.0, self.publish_twist)
    ```

    - `1.0`: The time interval in seconds.
    - `self.publish_twist`: The function that will be called every second.

    </details>

    <details>
    <summary>Hint 3 - Publishing the Twist Message</summary>

    After creating the `Twist` message, publish it using the publisher you created:

    ```python
    self.publisher_.publish(twist)
    ```

    This line sends the movement command to the turtle, making it move.

    </details>

    <details>
    <summary>Hint 4 - Full Solution</summary>

    If you’re still unsure, here’s the complete implementation for the `TurtleNode`:

    ```python
    #!/usr/bin/env python3

    import rclpy  # Import the ROS 2 Python client library
    from rclpy.node import Node  # Import the Node class from rclpy
    from geometry_msgs.msg import Twist  # Import the Twist message type for controlling the turtle

    class TurtleNode(Node): # Define a new class 'TurtleNode' that inherits from the ROS 2 'Node' class
        def __init__(self): # The constructor method to initialize the node's attributes and setup
            super().__init__('turtle_node')  # Initialize the node with the name 'turtle_node'
            
            # Create a publisher that publishes Twist messages to the '/turtle1/cmd_vel' topic
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

            # Set up a timer that periodically calls the publish_twist method
            self.timer = self.create_timer(1.0, self.publish_twist)

            self.get_logger().info("Turtle Node started!")  # Log a message to confirm the node has started

        def publish_twist(self):
            # Create and configure a Twist message to move the turtle
            twist = Twist()
            twist.linear.x = 2.0  # Set linear velocity in the x direction
            twist.angular.z = 1.0  # Set angular velocity around the z-axis (yaw)

            # Publish the Twist message
            self.publisher_.publish(twist)

            self.get_logger().info("Published twist message")  # Log a message after publishing the twist

    def main(args=None):
        rclpy.init(args=args)  # Initialize the ROS 2 Python client library
        node = TurtleNode()  # Create an instance of the TurtleNode
        rclpy.spin(node)  # Keep the node running until manually interrupted
        node.destroy_node()  # Clean up the node after shutdown
        rclpy.shutdown()  # Shutdown the ROS 2 client library

    if __name__ == '__main__':
        main()  # Run the main function if this script is executed directly
    ```

    </details>  

    <br>

3. **Build the Package:**

   Before running the node, ensure you are in the `osu-uwrt/development/software` directory and build the riptide_turtle package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

   If you get an error saying the package isn't there, try opening a new terminal window and running it. The same applies for the run commands below.

   **Note:** You will need to do this everytime you update a node file.

4. **Run the turtle sim:**

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

5. **Run the turtle node:**

   ```bash
   ros2 run riptide_turtle turtle_node.py
   ```

### Result

The turtle should move in a circle continuously as the node publishes `Twist` messages with a fixed linear and angular velocity.

---

## Step 2: Create a Kill Switch Node

### Objective

Create a separate ROS 2 node that publishes to the `/turtle_enabled` topic, enabling or disabling the turtle's movement.

### Guided Implementation

1. **Create a new Python script (`turtle_kill_switch.py`) in your ROS 2 package:**

   Below is the skeleton of your `TurtleEnablePublisher` class.
   - The skeleton is located in `src/turtle_kill_switch.py`
   - The topic you will need to publish to is `/turtle_enabled`
  
   Your task is to implement the missing functionality:

   ```python
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
   ```
  
   **Helpful Links:**

   - [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

   **Hints:**

   <details>
   <summary>Hint 1 - Creating the Publisher</summary>

   To publish `Bool` messages to enable/disable the turtle, you need to create a publisher. Here’s how you can do that:

   ```python
   self.publisher_ = self.create_publisher(Bool, 'turtle_enabled', 10)
   ```

   - `Bool`: The message type used to enable/disable the turtle.
   - `turtle_enabled`: The topic where the enable/disable state is published.
   - `10`: The queue size, which controls how many messages can be buffered.

   </details>

   <details>
   <summary>Hint 2 - Setting Up the Timers</summary>

   You need to periodically toggle the state and publish it. Here’s how to set up the timers:

   ```python
   self.state_timer = self.create_timer(30.0, self.toggle_state_callback)
   self.publish_timer = self.create_timer(1.0, self.publish_state_callback)
   ```

   - `30.0`: The time interval in seconds for toggling the state.
   - `1.0`: The time interval in seconds for publishing the state.

   </details>

   <details>
   <summary>Hint 3 - Publishing the Bool Message</summary>

   After creating the `Bool` message, publish it using the publisher you created:

   ```python
   self.publisher_.publish(msg)
   ```

   This line sends the enable/disable state to the turtle.

   </details>

   <details>
   <summary>Hint 4 - Full Solution</summary>

   If you’re still unsure, here’s the complete implementation for the `TurtleEnablePublisher`:

   ```python
   #!/usr/bin/env python3

   import rclpy  # Import the ROS 2 Python client library
   from rclpy.node import Node  # Import the Node class from rclpy
   from std_msgs.msg import Bool  # Import the Bool message type for enabling/disabling the turtle

   class TurtleEnablePublisher(Node):  # Define a new class 'TurtleEnablePublisher' that inherits from the ROS 2 'Node' class
       def __init__(self):  # The constructor method to initialize the node's attributes and setup
           super().__init__('turtle_kill_switch_publisher')  # Initialize the node with the name 'turtle_kill_switch_publisher'
           
           # Create a publisher that publishes Bool messages to the 'turtle_enabled' topic
           self.publisher_ = self.create_publisher(Bool, 'turtle_enabled', 10)

           # Set up a timer that periodically calls the toggle_state_callback method every 30 seconds
           self.state_timer = self.create_timer(30.0, self.toggle_state_callback)

           # Set up a timer that periodically calls the publish_state_callback method every second
           self.publish_timer = self.create_timer(1.0, self.publish_state_callback)

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

           # Publish the Bool message
           self.publisher_.publish(msg)

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
   ```

   </details>

   <br>

3. **Build the Package:**

   Before running the node, ensure you are in the `osu-uwrt/development/software` directory and rebuild the `riptide_turtle` package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

   **Note:** You will need to do this every time you update a node file.

4. **Run the Kill Switch Node:**

   ```bash
   ros2 run riptide_turtle turtle_kill_switch.py
   ```

5. **Check if the Kill Switch is Publishing:**

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

   Below is the updated `TurtleNode` class.
   - The topic you will need to subscribe to is `turtle_enabled`
  
   Your task is to implement the changes to subscribe to the `/turtle_enabled` topic and only publish `Twist` messages when the turtle is enabled:

   ```python
   #!/usr/bin/env python3

   import rclpy  # Import the ROS 2 Python client library
   from rclpy.node import Node  # Import the Node class from rclpy
   from geometry_msgs.msg import Twist  # Import the Twist message type for controlling the turtle
   from std_msgs.msg import Bool  # Import the Bool message type for enabling/disabling the turtle

   class TurtleNode(Node):  # Define a new class 'TurtleNode' that inherits from the ROS 2 'Node' class
       def __init__(self):  # The constructor method to initialize the node's attributes and setup
           super().__init__('turtle_node')  # Initialize the node with the name 'turtle_node'
           self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Create a publisher for Twist messages
           
           # TODO: Create a subscription to listen to the 'turtle_enabled' topic
           # self.subscription = ...

           self.timer = self.create_timer(1.0, self.publish_twist)  # Create a timer to publish Twist messages periodically
           self.enabled = False  # Initialize the enabled state as False (disabled)
           self.get_logger().info("Turtle Node started!")  # Log a message to confirm the node has started

       def enabled_callback(self, msg):
           # Update the enabled state based on the received Bool message
           self.enabled = msg.data
           state_str = "Enabled" if self.enabled else "Disabled"
           self.get_logger().info(f"Received state: {state_str}")

       def publish_twist(self):
           # TODO: Only publish the twist if the enabled state is True
           # if ...
               twist = Twist()
               twist.linear.x = 2.0  # Set linear velocity in the x direction
               twist.angular.z = 1.0  # Set angular velocity around the z-axis (yaw)
               self.publisher_.publish(twist)  # Publish the Twist message
               self.get_logger().info("Published twist message")

   def main(args=None):
       rclpy.init(args=args)  # Initialize the ROS 2 Python client library
       node = TurtleNode()  # Create an instance of the TurtleNode
       rclpy.spin(node)  # Keep the node running until manually interrupted
       node.destroy_node()  # Clean up the node after shutdown
       rclpy.shutdown()  # Shutdown the ROS 2 client library

   if __name__ == '__main__':
       main()  # Run the main function if this script is executed directly
   ```
   
   **Helpful Links:**

   - [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

   **Hints:**

   <details>
   <summary>Hint 1 - Creating the Subscription</summary>

   To subscribe to the `turtle_enabled` topic and control the turtle's movement, you need to create a subscription. Here’s how you can do that:

   ```python
   self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)
   ```

   - `Bool`: The message type that represents the enable/disable state.
   - `turtle_enabled`: The topic where the enable/disable state is published.
   - `self.enabled_callback`: The callback function to handle the received message.
   - `10`: The queue size, which controls how many messages can be buffered.

   </details>

   <details>
   <summary>Hint 2 - Using the Enabled State</summary>

   The turtle should only move if it is enabled. Modify the `publish_twist` method to check the `self.enabled` state before publishing a `Twist` message:

   ```python
   if self.enabled:
       twist = Twist()
       twist.linear.x = 2.0
       twist.angular.z = 1.0
       self.publisher_.publish(twist)
       self.get_logger().info("Published twist message")
   ```

   </details>

   <details>
   <summary>Hint 3 - Full Solution</summary>

   If you’re still unsure, here’s the complete implementation for the `TurtleNode` with the integrated kill switch:

   ```python
   #!/usr/bin/env python3

   import rclpy  # Import the ROS 2 Python client library
   from rclpy.node import Node  # Import the Node class from rclpy
   from geometry_msgs.msg import Twist  # Import the Twist message type for controlling the turtle
   from std_msgs.msg import Bool  # Import the Bool message type for enabling/disabling the turtle

   class TurtleNode(Node):  # Define a new class 'TurtleNode' that inherits from the ROS 2 'Node' class
       def __init__(self):  # The constructor method to initialize the node's attributes and setup
           super().__init__('turtle_node')  # Initialize the node with the name 'turtle_node'
           self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Create a publisher for Twist messages
           self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)  # Create a subscription to listen to the 'turtle_enabled' topic
           self.timer = self.create_timer(1.0, self.publish_twist)  # Create a timer to publish Twist messages periodically
           self.enabled = False  # Initialize the enabled state as False (disabled)
           self.get_logger().info("Turtle Node started!")  # Log a message to confirm the node has started

       def enabled_callback(self, msg):
           # Update the enabled state based on the received Bool message
           self.enabled = msg.data
           state_str = "Enabled" if self.enabled else "Disabled"
           self.get_logger().info(f"Received state: {state_str}")

       def publish_twist(self):
           # Only publish Twist messages if the turtle is enabled
           if self.enabled:
               twist = Twist()
               twist.linear.x = 2.0  # Set linear velocity in the x direction
               twist.angular.z = 1.0  # Set angular velocity around the z-axis (yaw)
               self.publisher_.publish(twist)  # Publish the Twist message
               self.get_logger().info("Published twist message")

   def main(args=None):
       rclpy.init(args=args)  # Initialize the ROS 2 Python client library
       node = TurtleNode()  # Create an instance of the TurtleNode
       rclpy.spin(node)  # Keep the node running until manually interrupted
       node.destroy_node()  # Clean up the node after shutdown
       rclpy.shutdown()  # Shutdown the ROS 2 client library

   if __name__ == '__main__':
       main()  # Run the main function if this script is executed directly
   ```

   </details>

   <br>

3. **Build the Package:**

   Before running the nodes, ensure you are in the `osu-uwrt/development/software` directory and again rebuild the `riptide_turtle` package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

4. **Run the nodes:**

   Start both the `TurtleNode` and `turtle_kill_switch` nodes in separate terminals:

   ```bash
   ros2 run riptide_turtle turtle_node.py
   ros2 run riptide_turtle turtle_kill_switch.py
   ```

### Result

The turtle will only move when the `/turtle_enabled` topic publishes `True`, as controlled by the `turtle_kill_switch` node.

---

## Step 4: Add Dynamic Parameters

### Objective

Introduce ROS 2 parameters to dynamically adjust the turtle's linear and angular speeds.

### Implementation

1. **Update the `TurtleNode` to use parameters for speed control:**

   Below is the updated `TurtleNode` class with added ROS 2 parameters to control the turtle’s speed dynamically.
   
  
   Your task is to implement the changes:

   ```python
    #!/usr/bin/env python3

    import rclpy  # Import the ROS 2 Python client library
    from rclpy.node import Node  # Import the Node class from rclpy
    from geometry_msgs.msg import Twist  # Import the Twist message type for controlling the turtle
    from std_msgs.msg import Bool  # Import the Bool message type for enabling/disabling the turtle

    class TurtleNode(Node):  # Define a new class 'TurtleNode' that inherits from the ROS 2 'Node' class
        def __init__(self):  # The constructor method to initialize the node's attributes and setup
            super().__init__('turtle_node')  # Initialize the node with the name 'turtle_node'
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Create a publisher for Twist messages
            self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)  # Create a subscription to listen to the 'turtle_enabled' topic
            self.timer = self.create_timer(1.0, self.publish_twist)  # Create a timer to publish Twist messages periodically

            # TODO: Declare parameters for linear and angular speed
            # self. ...
            # self. ...

            self.enabled = False  # Initialize the enabled state as False (disabled)
            self.get_logger().info("Turtle Node started!")  # Log a message to confirm the node has started

        def enabled_callback(self, msg):
            # Update the enabled state based on the received Bool message
            self.enabled = msg.data
            state_str = "Enabled" if self.enabled else "Disabled" # Make the Bool into a more readable state to log
            self.get_logger().info(f"Received state: {state_str}") # Log the state

        def publish_twist(self):
            # Only publish Twist messages if the turtle is enabled
            if self.enabled:
                # TODO: Get the current parameter values for speed
                # linear_speed = ...
                # angular_speed = ...

                twist = Twist()
                twist.linear.x = linear_speed  # Set linear velocity using the parameter value
                twist.angular.z = angular_speed  # Set angular velocity using the parameter value
                self.publisher_.publish(twist)  # Publish the Twist message
                self.get_logger().info(f"Published twist message with linear_speed={linear_speed}, angular_speed={angular_speed}")

    def main(args=None):
        rclpy.init(args=args)  # Initialize the ROS 2 Python client library
        node = TurtleNode()  # Create an instance of the TurtleNode
        rclpy.spin(node)  # Keep the node running until manually interrupted
        node.destroy_node()  # Clean up the node after shutdown
        rclpy.shutdown()  # Shutdown the ROS 2 client library

    if __name__ == '__main__':
        main()  # Run the main function if this script is executed directly
   ```

   **Helpful Links:**

   - [Using ROS 2 Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)

   **Hints:**

   <details>
   <summary>Hint 1 - Declaring Parameters</summary>

   To use parameters in your node, you need to declare them first. Here’s how you can declare parameters for controlling the turtle’s speed:

   ```python
   self.declare_parameter('linear_speed', 2.0)
   self.declare_parameter('angular_speed', 1.0)
   ```

   - `linear_speed`: The initial value for the linear speed of the turtle.
   - `angular_speed`: The initial value for the angular speed of the turtle.

   </details>

   <details>
   <summary>Hint 2 - Getting Parameter Values</summary>

   You can dynamically retrieve the values of these parameters using the `get_parameter` method:

   ```python
   linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
   angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
   ```

   This allows the turtle’s speed to be adjusted without restarting the node.

   </details>

   <details>
   <summary>Hint 3 - Full Solution</summary>

   If you’re still unsure, here’s the complete implementation for the `TurtleNode` with dynamic parameters:

    ```python
    #!/usr/bin/env python3

    import rclpy  # Import the ROS 2 Python client library
    from rclpy.node import Node  # Import the Node class from rclpy
    from geometry_msgs.msg import Twist  # Import the Twist message type for controlling the turtle
    from std_msgs.msg import Bool  # Import the Bool message type for enabling/disabling the turtle

    class TurtleNode(Node):  # Define a new class 'TurtleNode' that inherits from the ROS 2 'Node' class
        def __init__(self):  # The constructor method to initialize the node's attributes and setup
            super().__init__('turtle_node')  # Initialize the node with the name 'turtle_node'
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Create a publisher for Twist messages
            self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)  # Create a subscription to listen to the 'turtle_enabled' topic
            self.timer = self.create_timer(1.0, self.publish_twist)  # Create a timer to publish Twist messages periodically

            # Declare parameters for linear and angular speed
            self.declare_parameter('linear_speed', 2.0)
            self.declare_parameter('angular_speed', 1.0)

            self.enabled = False  # Initialize the enabled state as False (disabled)
            self.get_logger().info("Turtle Node started!")  # Log a message to confirm the node has started

        def enabled_callback(self, msg):
            # Update the enabled state based on the received Bool message
            self.enabled = msg.data
            state_str = "Enabled" if self.enabled else "Disabled" # Make the Bool into a more readable state to log
            self.get_logger().info(f"Received state: {state_str}") # Log the state

        def publish_twist(self):
            # Only publish Twist messages if the turtle is enabled
            if self.enabled:
                # Get the current parameter values for speed
                linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
                angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

                twist = Twist()
                twist.linear.x = linear_speed  # Set linear velocity using the parameter value
                twist.angular.z = angular_speed  # Set angular velocity using the parameter value
                self.publisher_.publish(twist)  # Publish the Twist message
                self.get_logger().info(f"Published twist message with linear_speed={linear_speed}, angular_speed={angular_speed}")

    def main(args=None):
        rclpy.init(args=args)  # Initialize the ROS 2 Python client library
        node = TurtleNode()  # Create an instance of the TurtleNode
        rclpy.spin(node)  # Keep the node running until manually interrupted
        node.destroy_node()  # Clean up the node after shutdown
        rclpy.shutdown()  # Shutdown the ROS 2 client library

    if __name__ == '__main__':
        main()  # Run the main function if this script is executed directly
    ```

   </details>

   <br>

2. **Build the Package:**

   Before running the node, ensure you are in the `osu-uwrt/development/software` directory and rebuild the `riptide_turtle` package:

   ```bash
   colcon build --packages-select riptide_turtle
   ```

3. **Restart the turtle node:**

   If the turtle node is currently running, stop it using `ctrl+c` in the terminal. Type `bash` and hit enter to start a new bash session, then restart the node:

   ```bash
   ros2 run riptide_turtle turtle_node.py
   ```

4. **Change the parameters dynamically:**

   You can now change the turtle’s speed dynamically using the following commands:

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

- The service you will need to create to is `/reset_speed`
- You will need the message type `Trigger` from `std_srvs.srv`

1. **Update the TurtleNode to include a reset speed service:**

    ```python
    #!/usr/bin/env python3

    import rclpy  # Import the ROS 2 Python client library
    from rclpy.node import Node  # Import the Node class from rclpy
    from geometry_msgs.msg import Twist  # Import the Twist message type for controlling the turtle
    from std_msgs.msg import Bool  # Import the Bool message type for enabling/disabling the turtle
    # TODO: Import the Trigger message type

    class TurtleNode(Node):  # Define a new class 'TurtleNode' that inherits from the ROS 2 'Node' class
        def __init__(self):  # The constructor method to initialize the node's attributes and setup
            super().__init__('turtle_node')  # Initialize the node with the name 'turtle_node'
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Create a publisher for Twist messages
            self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)  # Create a subscription to listen to the 'turtle_enabled' topic
            self.timer = self.create_timer(1.0, self.publish_twist)  # Create a timer to publish Twist messages periodically

            # TODO: Create a service named 'reset_speed' that calls 'reset_speed_callback' when triggered
            # self.reset_speed_service = ...

            # Declare parameters for linear and angular speed
            self.declare_parameter('linear_speed', 2.0)
            self.declare_parameter('angular_speed', 1.0)

            self.enabled = False  # Initialize the enabled state as False (disabled)
            self.get_logger().info("Turtle Node started!")  # Log a message to confirm the node has started

        def enabled_callback(self, msg):
            # Update the enabled state based on the received Bool message
            self.enabled = msg.data
            state_str = "Enabled" if self.enabled else "Disabled" # Make the Bool into a more readable state to log
            self.get_logger().info(f"Received state: {state_str}") # Log the state

        def publish_twist(self):
            # Only publish Twist messages if the turtle is enabled
            if self.enabled:
                # Get the current parameter values for speed
                linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
                angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

                twist = Twist()
                twist.linear.x = linear_speed  # Set linear velocity using the parameter value
                twist.angular.z = angular_speed  # Set angular velocity using the parameter value
                self.publisher_.publish(twist)  # Publish the Twist message
                self.get_logger().info(f"Published twist message with linear_speed={linear_speed}, angular_speed={angular_speed}")
        
        def reset_speed_callback(self, request, response):
            self.set_parameters([
                # TODO: Reset the 'linear_speed' and 'angular_speed' parameters to their default values
                # rclpy.parameter.Parameter(...),
                # rclpy.parameter.Parameter(...)
            ])
            self.get_logger().info("Linear and Angular speeds reset to default values")
            response.success = True  # Indicate that the service call was successful
            response.message = "Speeds have been reset to default values"  # Provide a confirmation message for the service response
            return response  # Return the response object to complete the service callback

    def main(args=None):
        rclpy.init(args=args)  # Initialize the ROS 2 Python client library
        node = TurtleNode()  # Create an instance of the TurtleNode
        rclpy.spin(node)  # Keep the node running until manually interrupted
        node.destroy_node()  # Clean up the node after shutdown
        rclpy.shutdown()  # Shutdown the ROS 2 client library

    if __name__ == '__main__':
        main()  # Run the main function if this script is executed directly
    ```

   
    **Helpful Links:**

   - [Using ROS 2 Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
   - [Writing Services and Clients](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)

   **Hints:**
   <details>
   <summary>Hint 1 - Creating a Service</summary>

   To create a service in ROS 2, you can use the `create_service` method, specifying the service type (like `Trigger`) and the callback function to handle service requests.

   Example:

   ```python
   self.create_service(Trigger, 'reset_speed', self.reset_speed_callback)
   ```

   </details>

   <details>
   <summary>Hint 2 - Setting Parameters</summary>

   You can reset parameters by using the `set_parameters` method, which allows you to set multiple parameters at once. 

   Example:

   ```python
   self.set_parameters([
       rclpy.parameter.Parameter('linear_speed', rclpy.Parameter.Type.DOUBLE, 2.0),
       rclpy.parameter.Parameter('angular_speed', rclpy.Parameter.Type.DOUBLE, 1.0)
   ])
   ```

   </details>

   <details>
   <summary>Hint 3 - Full Solution</summary>

   If you’re unsure, here’s the complete implementation with the service included:

   ```python
   #!/usr/bin/env python3

    import rclpy  # Import the ROS 2 Python client library
    from rclpy.node import Node  # Import the Node class from rclpy
    from geometry_msgs.msg import Twist  # Import the Twist message type for controlling the turtle
    from std_msgs.msg import Bool  # Import the Bool message type for enabling/disabling the turtle
    from std_srvs.srv import Trigger # Import the trigger message type for the service

    class TurtleNode(Node):  # Define a new class 'TurtleNode' that inherits from the ROS 2 'Node' class
        def __init__(self):  # The constructor method to initialize the node's attributes and setup
            super().__init__('turtle_node')  # Initialize the node with the name 'turtle_node'
            self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)  # Create a publisher for Twist messages
            self.subscription = self.create_subscription(Bool, 'turtle_enabled', self.enabled_callback, 10)  # Create a subscription to listen to the 'turtle_enabled' topic
            self.timer = self.create_timer(1.0, self.publish_twist)  # Create a timer to publish Twist messages periodically

            self.create_service(Trigger, 'reset_speed', self.reset_speed_callback) # Create a service to reset the speed

            # Declare parameters for linear and angular speed
            self.declare_parameter('linear_speed', 2.0)
            self.declare_parameter('angular_speed', 1.0)

            self.enabled = False  # Initialize the enabled state as False (disabled)
            self.get_logger().info("Turtle Node started!")  # Log a message to confirm the node has started

        def enabled_callback(self, msg):
            # Update the enabled state based on the received Bool message
            self.enabled = msg.data
            state_str = "Enabled" if self.enabled else "Disabled" # Make the Bool into a more readable state to log
            self.get_logger().info(f"Received state: {state_str}") # Log the state

        def publish_twist(self):
            # Only publish Twist messages if the turtle is enabled
            if self.enabled:
                # Get the current parameter values for speed
                linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
                angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

                twist = Twist()
                twist.linear.x = linear_speed  # Set linear velocity using the parameter value
                twist.angular.z = angular_speed  # Set angular velocity using the parameter value
                self.publisher_.publish(twist)  # Publish the Twist message
                self.get_logger().info(f"Published twist message with linear_speed={linear_speed}, angular_speed={angular_speed}")
        
        def reset_speed_callback(self, request, response):
            self.set_parameters([
                rclpy.parameter.Parameter('linear_speed', rclpy.Parameter.Type.DOUBLE, 2.0), # Set the linear speed parameter to 2.0
                rclpy.parameter.Parameter('angular_speed', rclpy.Parameter.Type.DOUBLE, 1.0) # Set the angular speed parameter to 1.0
            ])
            self.get_logger().info("Linear and Angular speeds reset to default values")
            response.success = True  # Indicate that the service call was successful
            response.message = "Speeds have been reset to default values"  # Provide a confirmation message for the service response
            return response  # Return the response object to complete the service callback

    def main(args=None):
        rclpy.init(args=args)  # Initialize the ROS 2 Python client library
        node = TurtleNode()  # Create an instance of the TurtleNode
        rclpy.spin(node)  # Keep the node running until manually interrupted
        node.destroy_node()  # Clean up the node after shutdown
        rclpy.shutdown()  # Shutdown the ROS 2 client library

    if __name__ == '__main__':
        main()  # Run the main function if this script is executed directly
   ```

   </details>

3. **Rebuild the Package:**

    Before running the node, ensure you are in the `osu-uwrt/development/software` directory and rebuild the riptide_turtle package:

    ```bash
    colcon build --packages-select riptide_turtle
    ```

4. **Restart the turtle node:**

    If the turtle node is currently running, ctrl+c in the terminal to stop it. Type `bash` and hit enter to start a new bash session, then restart the node:

    ```bash
    ros2 run riptide_turtle turtle_node.py
    ```

5. **Call the service to reset speeds:**

    ```bash
    ros2 service call /reset_speed std_srvs/srv/Trigger
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
