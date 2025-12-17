---
sidebar_position: 8
---

# Python-based ROS 2 Nodes

## Understanding Python Nodes in ROS 2

Python-based ROS 2 nodes are executable programs written in Python that participate in the ROS 2 ecosystem. They can publish and subscribe to topics, provide and call services, and manage parameters. These nodes are particularly valuable for AI agents due to Python's strong ecosystem for machine learning and artificial intelligence.

## Creating a Complete Python Node

Let's build a comprehensive example of a Python-based ROS 2 node that demonstrates multiple capabilities:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)

        # Subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Services
        self.enable_service = self.create_service(
            SetBool,
            'enable_controller',
            self.enable_callback
        )

        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('safety_distance', 0.5)

        # Internal state
        self.controller_enabled = True
        self.obstacle_detected = False

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Robot Controller Node initialized')

    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles"""
        # Find minimum distance in the front 90-degree sector
        front_distances = msg.ranges[315:360] + msg.ranges[0:45]  # Front 90 degrees

        # Filter out invalid readings (inf, nan)
        valid_distances = [d for d in front_distances if d != float('inf') and not d != d]  # not d != d checks for nan

        if valid_distances:
            min_distance = min(valid_distances)
            self.obstacle_detected = min_distance < self.get_parameter('safety_distance').value
        else:
            self.obstacle_detected = False

    def enable_callback(self, request, response):
        """Service callback to enable/disable controller"""
        self.controller_enabled = request.data
        response.success = True
        response.message = f'Controller {"enabled" if self.controller_enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def control_loop(self):
        """Main control loop executed periodically"""
        if not self.controller_enabled:
            # Stop robot if controller is disabled
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            return

        cmd = Twist()

        if self.obstacle_detected:
            # Rotate in place to avoid obstacle
            cmd.linear.x = 0.0
            cmd.angular.z = self.get_parameter('angular_speed').value
            status_msg = String()
            status_msg.data = 'Obstacle detected, rotating'
            self.status_publisher.publish(status_msg)
        else:
            # Move forward
            cmd.linear.x = self.get_parameter('linear_speed').value
            cmd.angular.z = 0.0
            status_msg = String()
            status_msg.data = 'Moving forward'
            self.status_publisher.publish(status_msg)

        # Publish command
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Architecture Patterns

### 1. Publisher Node Pattern

A node that only publishes data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class SensorSimulatorNode(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        self.publisher = self.create_publisher(Float64MultiArray, 'sensor_data', 10)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)
        self.time = 0.0

    def publish_sensor_data(self):
        msg = Float64MultiArray()
        # Simulate sensor data (e.g., joint positions)
        joint_positions = [
            math.sin(self.time),
            math.cos(self.time),
            math.sin(self.time * 2)
        ]
        msg.data = joint_positions
        self.publisher.publish(msg)
        self.time += 0.1
```

### 2. Subscriber Node Pattern

A node that only processes incoming data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class JointStateMonitorNode(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        self.status_publisher = self.create_publisher(String, 'system_status', 10)

    def joint_state_callback(self, msg):
        # Check if any joint is out of safe limits
        for i, position in enumerate(msg.position):
            if abs(position) > 3.0:  # Example safety limit
                status_msg = String()
                status_msg.data = f'Joint {msg.name[i]} out of safe limits: {position}'
                self.status_publisher.publish(status_msg)
                self.get_logger().warn(status_msg.data)
```

### 3. Service Server Node Pattern

A node that provides specific functionality:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger
from std_msgs.msg import String

class TaskExecutorNode(Node):
    def __init__(self):
        super().__init__('task_executor')
        self.srv = self.create_service(
            Trigger,
            'execute_task',
            self.execute_task_callback
        )
        self.status_publisher = self.create_publisher(String, 'task_status', 10)

    def execute_task_callback(self, request, response):
        # Simulate task execution
        self.get_logger().info('Executing task...')

        # Publish task status
        status_msg = String()
        status_msg.data = 'Task started'
        self.status_publisher.publish(status_msg)

        # Simulate some work (in real implementation, this might be async)
        import time
        time.sleep(1)  # Simulate work

        status_msg.data = 'Task completed'
        self.status_publisher.publish(status_msg)

        response.success = True
        response.message = 'Task completed successfully'
        return response
```

## Advanced Node Features

### 1. Using Callback Groups

For more complex nodes with multiple threads:

```python
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class MultiThreadedNode(Node):
    def __init__(self):
        super().__init__('multi_threaded_node')

        # Create separate callback groups
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        # Create subscriptions with different callback groups
        self.sub1 = self.create_subscription(
            String, 'topic1', self.callback1, 10, callback_group=self.group1
        )
        self.sub2 = self.create_subscription(
            String, 'topic2', self.callback2, 10, callback_group=self.group2
        )

        # Create timers with different callback groups
        self.timer1 = self.create_timer(1.0, self.timer1_callback, callback_group=self.group1)
        self.timer2 = self.create_timer(2.0, self.timer2_callback, callback_group=self.group2)

    def callback1(self, msg):
        self.get_logger().info(f'Callback 1 received: {msg.data}')

    def callback2(self, msg):
        self.get_logger().info(f'Callback 2 received: {msg.data}')

    def timer1_callback(self):
        self.get_logger().info('Timer 1 callback')

    def timer2_callback(self):
        self.get_logger().info('Timer 2 callback')

def main(args=None):
    rclpy.init(args=args)
    node = MultiThreadedNode()

    # Use multi-threaded executor to handle different callback groups
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 2. Node Composition

Creating multiple nodes within a single process:

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

class ComponentNode1(Node):
    def __init__(self):
        super().__init__('component1')
        self.pub = self.create_publisher(String, 'component1_out', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from Component 1'
        self.pub.publish(msg)

class ComponentNode2(Node):
    def __init__(self):
        super().__init__('component2')
        self.sub = self.create_subscription(
            String, 'component1_out', self.sub_callback, 10
        )
        self.pub = self.create_publisher(String, 'component2_out', 10)

    def sub_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # Forward message with modification
        output_msg = String()
        output_msg.data = f'Processed: {msg.data}'
        self.pub.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)

    # Create multiple nodes
    node1 = ComponentNode1()
    node2 = ComponentNode2()

    # Use single executor to run both nodes
    executor = SingleThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)

    try:
        executor.spin()
    finally:
        node1.destroy_node()
        node2.destroy_node()
        rclpy.shutdown()
```

## Best Practices for Python Nodes

### 1. Resource Management
Always properly clean up resources:

```python
class WellDesignedNode(Node):
    def __init__(self):
        super().__init__('well_designed_node')
        self._publishers = []
        self._subscribers = []
        self._timers = []
        self._services = []
        self._clients = []

        # Create resources
        pub = self.create_publisher(String, 'topic', 10)
        self._publishers.append(pub)

    def destroy_node(self):
        # Clean up all resources
        for pub in self._publishers:
            pub.destroy()
        for sub in self._subscribers:
            sub.destroy()
        for timer in self._timers:
            timer.destroy()
        for srv in self._services:
            srv.destroy()
        for client in self._clients:
            client.destroy()

        super().destroy_node()
```

### 2. Error Handling
Implement proper error handling in callbacks:

```python
def robust_callback(self, msg):
    try:
        # Process message
        result = self.process_message(msg)
        self.publish_result(result)
    except ValueError as e:
        self.get_logger().error(f'Invalid message format: {e}')
    except Exception as e:
        self.get_logger().error(f'Unexpected error: {e}')
        # Don't let callback crash the node
```

### 3. Performance Considerations
For performance-critical applications:

```python
class PerformanceNode(Node):
    def __init__(self):
        super().__init__('performance_node')

        # Use appropriate QoS for performance
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

        qos_profile = QoSProfile(
            depth=1,  # Minimal queue depth for lowest latency
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.fast_sub = self.create_subscription(
            String, 'fast_topic', self.fast_callback, qos_profile
        )
```

## Summary

Python-based ROS 2 nodes provide a powerful way to implement complex robotic behaviors:

- **Complete nodes** combine publishers, subscribers, services, and parameters
- **Design patterns** help structure different types of nodes
- **Advanced features** like callback groups and composition enable complex architectures
- **Best practices** ensure robust and maintainable code

These capabilities make Python an excellent choice for implementing AI agents that interact with robotic systems through ROS 2.