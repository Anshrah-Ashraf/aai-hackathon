---
sidebar_position: 10
---

# Agent-to-Actuator Communication

## Overview of Agent-to-Actuator Communication

Agent-to-actuator communication is the process by which AI agents send commands to physical actuators in robotic systems. This communication must be reliable, timely, and safe to ensure proper robot operation. Understanding this communication pattern is crucial for implementing effective AI control of robotic systems.

## Communication Architectures

### 1. Direct Communication Pattern

In the direct pattern, the AI agent communicates directly with actuator controllers:

```
┌─────────────────┐    ROS 2    ┌─────────────────┐
│   AI Agent      │────────────►│ Actuator        │
│  (High-level)   │             │ Controllers     │
└─────────────────┘             └─────────────────┘
```

**Advantages:**
- Low latency
- Simple architecture
- Direct control

**Disadvantages:**
- AI agent needs detailed knowledge of actuators
- Less flexibility in control strategies
- Potential safety concerns

### 2. Hierarchical Communication Pattern

In the hierarchical pattern, communication flows through intermediate controllers:

```
┌─────────────────┐    High-level    ┌─────────────────┐
│   AI Agent      │─────────────────►│ Task Planner    │
│  (Behavior)     │                  │ (Motion)        │
└─────────────────┘                  └─────────────────┘
                                               │
                                        ROS 2  │
                                               ▼
┌─────────────────┐                  ┌─────────────────┐
│ Hardware        │◄─────────────────┤ Joint Controller│
│ Interface       │    Low-level     │ (Low-level)     │
└─────────────────┘                  └─────────────────┘
```

**Advantages:**
- Better separation of concerns
- More robust to actuator failures
- Easier to implement safety measures

**Disadvantages:**
- Higher latency
- More complex architecture
- Potential for communication bottlenecks

## Message Types for Actuator Control

### 1. Joint Trajectory Messages

For controlling robot joints with precise trajectories:

```python
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal
import rclpy
from rclpy.node import Node

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control')
        self.joint_pub = self.create_publisher(JointTrajectory, '/joint_trajectory', 10)

    def send_joint_trajectory(self, joint_names, positions, velocities=None, time_from_start=5.0):
        """Send a joint trajectory command"""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(time_from_start)
        point.time_from_start.nanosec = int((time_from_start - int(time_from_start)) * 1e9)

        if velocities:
            point.velocities = velocities

        traj_msg.points = [point]
        self.joint_pub.publish(traj_msg)

    def example_usage(self):
        """Example of sending joint commands"""
        joint_names = ['joint1', 'joint2', 'joint3']
        positions = [1.0, 0.5, -0.3]  # Radians
        velocities = [0.1, 0.1, 0.1]  # Optional: joint velocities

        self.send_joint_trajectory(joint_names, positions, velocities)
```

### 2. Velocity Commands

For continuous velocity control:

```python
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class VelocityControlNode(Node):
    def __init__(self):
        super().__init__('velocity_control')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_velocity_command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Send velocity command to robot"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.linear.z = linear_z
        cmd.angular.x = angular_x
        cmd.angular.y = angular_y
        cmd.angular.z = angular_z

        self.cmd_pub.publish(cmd)

    def move_forward(self, speed=0.5):
        """Move robot forward at specified speed"""
        self.send_velocity_command(speed, 0.0, 0.0, 0.0, 0.0, 0.0)

    def rotate(self, angular_speed=0.5):
        """Rotate robot at specified angular speed"""
        self.send_velocity_command(0.0, 0.0, 0.0, 0.0, 0.0, angular_speed)
```

### 3. Position Commands

For direct position control:

```python
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node

class PositionControlNode(Node):
    def __init__(self):
        super().__init__('position_control')
        self.position_pub = self.create_publisher(Float64MultiArray, '/position_commands', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.current_positions = {}

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        self.current_positions = dict(zip(msg.name, msg.position))

    def send_position_command(self, joint_positions):
        """Send position commands to actuators"""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = list(joint_positions.values())

        # Optionally include joint names as layout
        # cmd_msg.layout.dim.append(MultiArrayDimension())
        # cmd_msg.layout.dim[0].label = ','.join(joint_positions.keys())
        # cmd_msg.layout.dim[0].size = len(joint_positions)
        # cmd_msg.layout.dim[0].stride = len(joint_positions)

        self.position_pub.publish(cmd_msg)

    def move_to_pose(self, joint_angles):
        """Move all specified joints to target angles"""
        self.send_position_command(joint_angles)
```

## Communication Patterns and Best Practices

### 1. Command Validation Pattern

Validate commands before sending to actuators:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ValidatedControlNode(Node):
    def __init__(self):
        super().__init__('validated_control')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/validated_cmd_vel', 10)

        # Parameters for validation
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('min_position', -2.0)
        self.declare_parameter('max_position', 2.0)

    def send_validated_command(self, cmd):
        """Send command after validation"""
        validated_cmd = self.validate_command(cmd)
        if validated_cmd:
            self.cmd_pub.publish(validated_cmd)
            return True
        else:
            self.get_logger().warn('Command validation failed')
            return False

    def validate_command(self, cmd):
        """Validate command against safety limits"""
        # Check linear velocity limits
        max_linear = self.get_parameter('max_linear_speed').value
        if abs(cmd.linear.x) > max_linear:
            cmd.linear.x = max_linear if cmd.linear.x > 0 else -max_linear

        # Check angular velocity limits
        max_angular = self.get_parameter('max_angular_speed').value
        if abs(cmd.angular.z) > max_angular:
            cmd.angular.z = max_angular if cmd.angular.z > 0 else -max_angular

        # Additional validation could include:
        # - Collision checks
        # - Joint limit checks
        # - Dynamic constraints
        # - Safety zone validation

        return cmd
```

### 2. Feedback-Driven Control

Implement control loops with actuator feedback:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header
import time

class FeedbackControlNode(Node):
    def __init__(self):
        super().__init__('feedback_control')

        # Publishers and subscribers
        self.command_pub = self.create_publisher(Float64MultiArray, '/position_commands', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Control parameters
        self.target_positions = {}
        self.current_positions = {}
        self.tolerance = 0.01  # 1cm tolerance

        # PID controller parameters
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.05  # Derivative gain

        self.prev_error = {}
        self.integral_error = {}

    def joint_callback(self, msg):
        """Update current joint positions"""
        self.current_positions = dict(zip(msg.name, msg.position))

    def move_to_position(self, joint_positions):
        """Move joints to target positions with feedback control"""
        self.target_positions = joint_positions

        # Initialize error accumulators
        for joint in joint_positions.keys():
            if joint not in self.prev_error:
                self.prev_error[joint] = 0.0
                self.integral_error[joint] = 0.0

        # Control loop until target reached
        reached_target = False
        while not reached_target and rclpy.ok():
            reached_target = self.execute_control_step()
            time.sleep(0.01)  # 100Hz control loop

    def execute_control_step(self):
        """Execute one step of feedback control"""
        commands = {}

        for joint, target_pos in self.target_positions.items():
            if joint in self.current_positions:
                current_pos = self.current_positions[joint]
                error = target_pos - current_pos

                # Check if within tolerance
                if abs(error) <= self.tolerance:
                    commands[joint] = 0.0  # No movement needed
                    continue

                # PID control calculation
                self.integral_error[joint] += error * 0.01  # dt = 0.01s
                derivative = (error - self.prev_error[joint]) / 0.01

                output = (self.kp * error +
                         self.ki * self.integral_error[joint] +
                         self.kd * derivative)

                # Limit output to reasonable range
                output = max(-1.0, min(1.0, output))

                commands[joint] = output
                self.prev_error[joint] = error

        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = list(commands.values())
        self.command_pub.publish(cmd_msg)

        # Check if all joints are within tolerance
        all_reached = True
        for joint, target_pos in self.target_positions.items():
            if joint in self.current_positions:
                error = abs(target_pos - self.current_positions[joint])
                if error > self.tolerance:
                    all_reached = False
                    break

        return all_reached
```

### 3. Asynchronous Command Execution

Handle long-running actuator commands:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
import threading
import time

class AsyncControlNode(Node):
    def __init__(self):
        super().__init__('async_control')

        # Action client for complex trajectories
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, 'follow_joint_trajectory'
        )

        # Publishers for simple commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Track active commands
        self.active_commands = {}
        self.command_counter = 0

    def send_trajectory_async(self, trajectory_msg):
        """Send trajectory command asynchronously"""
        self.trajectory_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg

        # Generate unique command ID
        cmd_id = self.command_counter
        self.command_counter += 1

        # Send goal and store tracking info
        future = self.trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self.trajectory_feedback_callback
        )

        # Store command tracking
        self.active_commands[cmd_id] = {
            'future': future,
            'start_time': self.get_clock().now(),
            'type': 'trajectory'
        }

        # Handle response asynchronously
        future.add_done_callback(lambda f: self.trajectory_done_callback(f, cmd_id))

        return cmd_id

    def trajectory_feedback_callback(self, feedback_msg):
        """Handle trajectory execution feedback"""
        self.get_logger().debug(f'Trajectory feedback: {feedback_msg}')

    def trajectory_done_callback(self, future, cmd_id):
        """Handle completion of trajectory command"""
        try:
            goal_handle = future.result()
            result = goal_handle.get_result_async()
            result.add_done_callback(lambda r: self.process_result(r, cmd_id))
        except Exception as e:
            self.get_logger().error(f'Error in trajectory command {cmd_id}: {e}')
            del self.active_commands[cmd_id]

    def process_result(self, result_future, cmd_id):
        """Process the final result of a command"""
        try:
            result = result_future.result().result
            self.get_logger().info(f'Command {cmd_id} completed with result: {result}')
        except Exception as e:
            self.get_logger().error(f'Error processing result for command {cmd_id}: {e}')
        finally:
            if cmd_id in self.active_commands:
                del self.active_commands[cmd_id]

    def is_command_active(self, cmd_id):
        """Check if a command is still active"""
        return cmd_id in self.active_commands
```

## Safety and Error Handling

### 1. Graceful Degradation

Handle actuator failures gracefully:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String

class SafeControlNode(Node):
    def __init__(self):
        super().__init__('safe_control')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/control_status', 10)

        # Subscribers
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Actuator status tracking
        self.actuator_status = {}
        self.faulty_actuators = set()

    def joint_callback(self, msg):
        """Monitor actuator status"""
        for i, name in enumerate(msg.name):
            # Check for NaN or extreme values (indicating faults)
            if i < len(msg.position) and (msg.position[i] != msg.position[i] or  # Check for NaN
                                         abs(msg.position[i]) > 1000):  # Extreme value check
                if name not in self.faulty_actuators:
                    self.faulty_actuators.add(name)
                    self.get_logger().error(f'Actuator fault detected: {name}')
                    self.publish_status(f'FAULT: {name}')

    def send_command_with_fallback(self, cmd):
        """Send command with fallback for faulty actuators"""
        if self.faulty_actuators:
            # Modify command to account for faulty actuators
            safe_cmd = self.compute_safe_command(cmd)
        else:
            safe_cmd = cmd

        self.cmd_pub.publish(safe_cmd)

    def compute_safe_command(self, original_cmd):
        """Compute safe command when some actuators are faulty"""
        safe_cmd = Twist()

        # Reduce speeds when actuators are faulty
        if self.faulty_actuators:
            safe_cmd.linear.x = original_cmd.linear.x * 0.5  # Reduce speed by 50%
            safe_cmd.angular.z = original_cmd.angular.z * 0.5
            self.get_logger().warn('Reducing speed due to actuator faults')
        else:
            safe_cmd.linear.x = original_cmd.linear.x
            safe_cmd.angular.z = original_cmd.angular.z

        return safe_cmd

    def publish_status(self, status_msg):
        """Publish control system status"""
        msg = String()
        msg.data = status_msg
        self.status_pub.publish(msg)
```

## Performance Considerations

### 1. Bandwidth Optimization

Optimize communication for efficient bandwidth usage:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class OptimizedControlNode(Node):
    def __init__(self):
        super().__init__('optimized_control')

        # Publishers with optimized QoS
        from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

        # For control commands - prioritize reliability
        cmd_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # For sensor data - prioritize latest data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', cmd_qos)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, sensor_qos)

        # Command filtering to reduce unnecessary updates
        self.last_command = None
        self.command_threshold = 0.01  # Only send if change is significant

    def send_command_if_changed(self, cmd):
        """Send command only if it differs significantly from last command"""
        if self.last_command is None or self.command_changed_significantly(cmd, self.last_command):
            self.cmd_pub.publish(cmd)
            self.last_command = cmd
            return True
        return False

    def command_changed_significantly(self, cmd1, cmd2):
        """Check if command changed significantly"""
        linear_diff = abs(cmd1.linear.x - cmd2.linear.x)
        angular_diff = abs(cmd1.angular.z - cmd2.angular.z)
        return linear_diff > self.command_threshold or angular_diff > self.command_threshold

    def joint_callback(self, msg):
        """Process joint states with data compression if needed"""
        # Process only when needed, not every message
        # This reduces computational load
        pass
```

## Summary

Agent-to-actuator communication is a critical component of AI-controlled robotic systems:

- **Communication architectures** determine how commands flow from AI to actuators
- **Message types** include joint trajectories, velocity commands, and position commands
- **Validation patterns** ensure commands are safe before execution
- **Feedback control** enables precise actuator positioning
- **Asynchronous execution** handles long-running commands
- **Safety mechanisms** protect against failures and errors
- **Performance optimization** ensures efficient communication

Understanding these concepts enables the creation of robust, safe, and efficient AI-robot interfaces.