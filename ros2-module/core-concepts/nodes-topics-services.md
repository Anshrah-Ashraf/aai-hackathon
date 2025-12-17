---
sidebar_position: 4
---

# Nodes, Topics, and Services

## Core Communication Patterns

ROS 2 provides three primary communication patterns that form the backbone of robotic systems: **Nodes**, **Topics**, and **Services**. Understanding these concepts is crucial for building distributed robotic applications.

## Nodes

### What is a Node?

A **Node** is a process that performs computation in the ROS 2 system. It's the fundamental unit of computation in ROS 2. Each node typically performs a specific task and communicates with other nodes through topics and services.

### Node Characteristics

- **Single purpose**: Each node should have a well-defined, single responsibility
- **Communication**: Nodes communicate with other nodes through topics and services
- **Identity**: Each node has a unique name within the ROS 2 graph
- **Lifecycle**: Nodes can be started, stopped, and managed independently

### Node Example

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

## Topics

### What is a Topic?

A **Topic** is a named bus over which nodes exchange messages. It implements a publisher-subscriber communication pattern where:

- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- Communication is **asynchronous** and **broadcast** in nature

### Topic Characteristics

- **Unidirectional**: Data flows from publisher to subscriber
- **Broadcast**: Multiple subscribers can receive the same message
- **Decoupled**: Publishers and subscribers don't need to know about each other
- **Typed**: Each topic has a specific message type

### Topic Communication Flow

```
[Publisher Node A] ──┐
                     ├─── Topic: /sensor_data ─── [Subscriber Node X]
[Publisher Node B] ──┘                           [Subscriber Node Y]
                                                [Subscriber Node Z]
```

### Topic Example

```python
# Publisher
publisher = node.create_publisher(String, 'topic_name', 10)

# Subscriber
subscriber = node.create_subscription(
    String,
    'topic_name',
    callback_function,
    10
)
```

## Services

### What is a Service?

A **Service** implements a request-response communication pattern. It consists of:

- **Service Server**: Waits for requests and sends responses
- **Service Client**: Sends requests and waits for responses

### Service Characteristics

- **Synchronous**: Client waits for response from server
- **Request-Response**: One-to-one communication pattern
- **Bidirectional**: Data flows in both directions (request and response)
- **Blocking**: Client is blocked until response is received

### Service Communication Flow

```
[Service Client] ── Request ──→ [Service Server]
[Service Client] ←─ Response ─── [Service Server]
```

### Service Example

```python
# Service Server
srv = node.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)

# Service Client
client = node.create_client(AddTwoInts, 'add_two_ints')
```

## Communication Pattern Comparison

| Aspect | Topics | Services |
|--------|--------|----------|
| Pattern | Publisher-Subscriber | Request-Response |
| Synchronization | Asynchronous | Synchronous |
| Communication | One-to-many | One-to-one |
| Data Direction | Unidirectional | Bidirectional |
| Use Case | Sensor data, continuous updates | Actions, calculations, state queries |

## Practical Applications in AI-Robotics

### Topics in AI-Robotics
- **Sensor data streams**: Camera images, LIDAR scans, IMU data
- **AI model outputs**: Detected objects, navigation commands
- **Robot state**: Joint positions, battery levels, system status

### Services in AI-Robotics
- **Action execution**: Move robot, grasp object, take snapshot
- **State queries**: Get current position, check system status
- **Configuration**: Set parameters, update settings

## Summary

Nodes, topics, and services form the fundamental communication architecture of ROS 2:

- **Nodes** are the computational units
- **Topics** enable asynchronous, broadcast communication
- **Services** enable synchronous, request-response communication

This architecture allows AI agents to communicate with robot controllers effectively, enabling the creation of sophisticated robotic systems where different components can be developed and maintained independently.