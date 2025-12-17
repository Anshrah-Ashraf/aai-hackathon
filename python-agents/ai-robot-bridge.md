---
sidebar_position: 9
---

# Bridging AI Logic to Robot Controllers

## The AI-Robot Interface

The bridge between AI logic and robot controllers is a critical component in modern robotic systems. This interface allows sophisticated AI algorithms to interact with physical robot hardware through the ROS 2 middleware. The bridge must handle data conversion, timing coordination, and safety considerations.

## Architecture of AI-Robot Bridge

### High-Level Overview

```
┌─────────────────┐    ROS 2    ┌─────────────────┐
│   AI Agent      │◄───────────►│  Bridge Node    │
│  (Decision Mgr) │             │                 │
└─────────────────┘             └─────────────────┘
                                          │
                                    ROS 2 │
                                          ▼
┌─────────────────┐               ┌─────────────────┐
│ Robot Controller│◄──────────────│  Hardware       │
│  (Low-level)    │               │  Interface      │
└─────────────────┘               └─────────────────┘
```

### Key Components

1. **AI Decision Module**: Implements high-level reasoning and planning
2. **Bridge Node**: Translates between AI outputs and robot commands
3. **Robot Controller**: Executes low-level control commands
4. **State Feedback**: Provides robot state to the AI module

## Implementing the Bridge Pattern

### Basic Bridge Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge')

        # AI to Bridge communication
        self.ai_command_sub = self.create_subscription(
            String, 'ai_commands', self.ai_command_callback, 10
        )

        # Bridge to Robot communication
        self.robot_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot to Bridge communication (feedback)
        self.robot_state_sub = self.create_subscription(
            JointState, '/joint_states', self.robot_state_callback, 10
        )

        # Bridge to AI communication (feedback)
        self.ai_feedback_pub = self.create_publisher(String, 'ai_feedback', 10)

        # Internal state
        self.current_robot_state = {}
        self.ai_goals = []
        self.active = True

        self.get_logger().info('AI-Robot Bridge initialized')

    def ai_command_callback(self, msg):
        """Process commands from AI agent"""
        command = msg.data
        self.get_logger().info(f'Received AI command: {command}')

        # Parse and validate AI command
        if self.active and self.parse_and_execute_command(command):
            feedback_msg = String()
            feedback_msg.data = f'Command executed: {command}'
            self.ai_feedback_pub.publish(feedback_msg)
        else:
            feedback_msg = String()
            feedback_msg.data = f'Command failed: {command}'
            self.ai_feedback_pub.publish(feedback_msg)

    def robot_state_callback(self, msg):
        """Process robot state feedback"""
        self.current_robot_state = dict(zip(msg.name, msg.position))
        self.get_logger().debug(f'Robot state updated: {self.current_robot_state}')

    def parse_and_execute_command(self, command):
        """Parse AI command and convert to robot action"""
        try:
            # Example command parsing
            if command.startswith('move_to:'):
                # Extract target position
                target = command.split(':')[1]
                self.execute_move_command(target)
                return True
            elif command.startswith('rotate:'):
                angle = float(command.split(':')[1])
                self.execute_rotate_command(angle)
                return True
            elif command == 'stop':
                self.execute_stop_command()
                return True
            else:
                self.get_logger().warn(f'Unknown command: {command}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error executing command {command}: {e}')
            return False

    def execute_move_command(self, target):
        """Execute movement command"""
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward at 0.5 m/s
        cmd.angular.z = 0.0
        self.robot_cmd_pub.publish(cmd)

    def execute_rotate_command(self, angle):
        """Execute rotation command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = angle  # Angular velocity
        self.robot_cmd_pub.publish(cmd)

    def execute_stop_command(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.robot_cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AIBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Bridge interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Advanced Bridge Patterns

### 1. State Machine Bridge

For complex AI behaviors that require state management:

```python
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotState(Enum):
    IDLE = 1
    NAVIGATING = 2
    MANIPULATING = 3
    AVOIDING_OBSTACLE = 4
    ERROR = 5

class StateMachineBridge(Node):
    def __init__(self):
        super().__init__('state_machine_bridge')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.ai_sub = self.create_subscription(String, 'ai_commands', self.ai_command_callback, 10)
        self.state_pub = self.create_publisher(String, 'robot_state', 10)

        # State management
        self.current_state = RobotState.IDLE
        self.previous_state = RobotState.IDLE
        self.obstacle_detected = False

        # State transition timer
        self.state_timer = self.create_timer(0.1, self.state_machine)

    def scan_callback(self, msg):
        """Process laser scan for obstacle detection"""
        # Check front 90 degrees for obstacles
        front_scan = msg.ranges[315:360] + msg.ranges[0:45]
        valid_distances = [d for d in front_scan if d != float('inf') and d == d]  # d == d checks for not NaN

        if valid_distances:
            min_distance = min(valid_distances)
            self.obstacle_detected = min_distance < 0.5  # 0.5m threshold

    def ai_command_callback(self, msg):
        """Handle high-level AI commands"""
        if msg.data == 'navigate':
            if self.current_state == RobotState.IDLE:
                self.current_state = RobotState.NAVIGATING
        elif msg.data == 'manipulate':
            if self.current_state in [RobotState.IDLE, RobotState.NAVIGATING]:
                self.current_state = RobotState.MANIPULATING
        elif msg.data == 'stop':
            self.current_state = RobotState.IDLE

    def state_machine(self):
        """Main state machine logic"""
        # Handle obstacle detection (highest priority transition)
        if self.obstacle_detected and self.current_state != RobotState.AVOIDING_OBSTACLE:
            self.previous_state = self.current_state
            self.current_state = RobotState.AVOIDING_OBSTACLE

        # Execute current state behavior
        cmd = Twist()

        if self.current_state == RobotState.IDLE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.current_state == RobotState.NAVIGATING:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
        elif self.current_state == RobotState.MANIPULATING:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # Additional manipulation commands would go here
        elif self.current_state == RobotState.AVOIDING_OBSTACLE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotate to avoid obstacle
        elif self.current_state == RobotState.ERROR:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # Publish command
        self.cmd_pub.publish(cmd)

        # Publish state for AI feedback
        state_msg = String()
        state_msg.data = self.current_state.name
        self.state_pub.publish(state_msg)
```

### 2. Action-Based Bridge

For long-running tasks that require feedback:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.server import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory

class ActionBridgeNode(Node):
    def __init__(self):
        super().__init__('action_bridge')

        # Create action clients for robot actions
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self.traj_client = ActionClient(
            self, FollowJointTrajectory, 'follow_joint_trajectory'
        )

        # Create action server for AI requests
        self.ai_action_server = ActionServer(
            self,
            NavigateToPose,  # Reuse navigation action type
            'ai_navigate_to_pose',
            self.execute_ai_navigation
        )

        self.get_logger().info('Action Bridge initialized')

    def execute_ai_navigation(self, goal_handle):
        """Execute navigation requested by AI"""
        self.get_logger().info('Received AI navigation request')

        # Convert AI goal to robot navigation goal
        robot_goal = NavigateToPose.Goal()
        robot_goal.pose = goal_handle.request.pose

        # Send goal to robot navigation system
        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(robot_goal)

        # Monitor progress and provide feedback
        while not future.done():
            # Provide feedback to AI
            feedback_msg = NavigateToPose.Feedback()
            feedback_msg.current_pose = PoseStamped()  # Would get actual pose
            goal_handle.publish_feedback(feedback_msg)

        result = future.result()
        goal_handle.succeed()

        result_msg = NavigateToPose.Result()
        result_msg.result = result.result
        return result_msg
```

## Safety Considerations

### 1. Safety Wrapper Pattern

Implement safety checks between AI and robot:

```python
class SafetyWrapperNode(Node):
    def __init__(self):
        super().__init__('safety_wrapper')

        # Internal publishers (not exposed to AI)
        self.internal_cmd_pub = self.create_publisher(Twist, '/internal_cmd_vel', 10)

        # External interfaces (to AI and robot)
        self.ai_cmd_sub = self.create_subscription(String, 'ai_safe_commands', self.ai_command_callback, 10)
        self.robot_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.emergency_stop_pub = self.create_publisher(String, '/emergency_stop', 1)

        # Safety parameters
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('safety_timeout', 5.0)

        self.last_command_time = self.get_clock().now()

    def ai_command_callback(self, msg):
        """Process AI command with safety checks"""
        current_time = self.get_clock().now()

        # Check for timeout
        if (current_time - self.last_command_time).nanoseconds / 1e9 > self.get_parameter('safety_timeout').value:
            self.emergency_stop()
            return

        self.last_command_time = current_time

        # Parse and validate command
        cmd = self.validate_command(msg.data)
        if cmd:
            self.robot_cmd_pub.publish(cmd)
        else:
            self.get_logger().error('Invalid command received from AI')
            self.emergency_stop()

    def validate_command(self, command_str):
        """Validate command against safety constraints"""
        try:
            # Parse command (example: "linear:0.5,angular:0.2")
            parts = command_str.split(',')
            linear = float(parts[0].split(':')[1])
            angular = float(parts[1].split(':')[1])

            # Apply safety limits
            max_linear = self.get_parameter('max_linear_speed').value
            max_angular = self.get_parameter('max_angular_speed').value

            if abs(linear) > max_linear or abs(angular) > max_angular:
                self.get_logger().warn(f'Command exceeds safety limits: linear={linear}, angular={angular}')
                return None

            # Create validated command
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            return cmd

        except Exception as e:
            self.get_logger().error(f'Error parsing command: {e}')
            return None

    def emergency_stop(self):
        """Emergency stop procedure"""
        stop_cmd = Twist()
        self.robot_cmd_pub.publish(stop_cmd)
        self.get_logger().error('EMERGENCY STOP ACTIVATED')
```

## Integration with Popular AI Frameworks

### 1. TensorFlow/PyTorch Integration

```python
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class AIVisionBridge(Node):
    def __init__(self):
        super().__init__('ai_vision_bridge')

        # Image processing
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Load AI model (example with placeholder)
        self.load_model()

    def load_model(self):
        """Load AI model for inference"""
        # This would load your actual model
        # Example: self.model = torch.load('path/to/model.pth')
        self.get_logger().info('AI model loaded')

    def image_callback(self, msg):
        """Process camera image with AI model"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run AI inference
            action = self.run_inference(cv_image)

            # Convert AI action to robot command
            cmd = self.ai_action_to_robot_command(action)
            self.cmd_pub.publish(cmd)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def run_inference(self, image):
        """Run AI model inference on image"""
        # Process image and return action
        # This is where your AI logic would run
        # Example: prediction = self.model(image)
        return "move_forward"  # Placeholder

    def ai_action_to_robot_command(self, action):
        """Convert AI action to robot command"""
        cmd = Twist()
        if action == "move_forward":
            cmd.linear.x = 0.3
        elif action == "turn_left":
            cmd.angular.z = 0.5
        elif action == "turn_right":
            cmd.angular.z = -0.5
        return cmd
```

## Summary

The bridge between AI logic and robot controllers is essential for creating intelligent robotic systems:

- **Basic bridges** translate between AI outputs and robot commands
- **State machine bridges** manage complex behaviors with multiple states
- **Action-based bridges** handle long-running tasks with feedback
- **Safety wrappers** ensure safe operation between AI and hardware
- **AI framework integration** enables direct use of machine learning models

These patterns enable AI agents to effectively control and interact with physical robots through the ROS 2 middleware.