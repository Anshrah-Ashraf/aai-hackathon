---
sidebar_position: 7
---

# rclpy Basics

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API to interact with ROS 2 concepts such as nodes, topics, services, parameters, and actions. This library allows Python developers to create ROS 2 nodes and integrate them into robotic systems.

## Core Concepts in rclpy

### 1. Initialization and Shutdown

Before creating any ROS 2 entities, you must initialize the rclpy library:

```python
import rclpy
from rclpy.node import Node

def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create and use nodes
    my_node = MyNode()

    # Spin the node to process callbacks
    rclpy.spin(my_node)

    # Clean up
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Creating a Node

All ROS 2 functionality in Python happens within a Node. A node should inherit from `rclpy.node.Node`:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        # Initialize the parent class with node name
        super().__init__('my_node_name')

        # Node-specific initialization code goes here
        self.get_logger().info('MyNode has been initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers in rclpy

Publishers allow nodes to send messages to topics:

```python
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'topic_name', 10)

        # Create a timer to publish messages periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

## Subscribers in rclpy

Subscribers allow nodes to receive messages from topics:

```python
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

class Listener(Node):
    def __init__(self):
        super().__init__('listener')

        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services in rclpy

Services enable request-response communication:

### Service Server
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create a service server
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {response.sum}')
        return response
```

### Service Client
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # Create a service client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Parameters in rclpy

Parameters allow runtime configuration of nodes:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('param_name', 'default_value')
        self.declare_parameter('threshold', 10)

        # Get parameter values
        param_value = self.get_parameter('param_name').value
        threshold_value = self.get_parameter('threshold').value

        self.get_logger().info(f'Parameter value: {param_value}')
```

## Timers in rclpy

Timers allow periodic execution of functions:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TimedNode(Node):
    def __init__(self):
        super().__init__('timed_node')

        # Create publisher
        self.publisher = self.create_publisher(String, 'timed_topic', 10)

        # Create timer that calls callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Timed message {self.counter}'
        self.publisher.publish(msg)
        self.counter += 1
```

## Working with Custom Message Types

To use custom message types, first define them in `.msg` files, then import and use:

```python
# Assuming you have a custom message RobotCommand.msg
from my_package_msgs.msg import RobotCommand
import rclpy
from rclpy.node import Node

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.publisher = self.create_publisher(RobotCommand, 'robot_command', 10)

    def send_command(self, command_type, value):
        msg = RobotCommand()
        msg.command_type = command_type
        msg.value = value
        self.publisher.publish(msg)
```

## Error Handling and Best Practices

### 1. Proper Resource Management
Always clean up resources when shutting down:

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 2. Exception Handling in Callbacks
Handle exceptions in callbacks to prevent node crashes:

```python
def safe_callback(self, msg):
    try:
        # Process message
        result = self.process_message(msg)
        self.publish_result(result)
    except Exception as e:
        self.get_logger().error(f'Error processing message: {e}')
```

## Summary

rclpy provides the essential tools for creating Python-based ROS 2 nodes:

- **Initialization**: Initialize rclpy before creating nodes
- **Nodes**: Inherit from `rclpy.node.Node` for ROS 2 functionality
- **Publishers/Subscribers**: Enable topic-based communication
- **Services**: Enable request-response communication
- **Parameters**: Enable runtime configuration
- **Timers**: Enable periodic execution

With these basics, you can create sophisticated Python agents that interact with robotic systems through ROS 2.