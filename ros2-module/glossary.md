---
sidebar_position: 15
---

# ROS2 Glossary

This glossary provides definitions for key terms used throughout the ROS2 Robotic Nervous System module.

## A

### Action
A communication pattern in ROS2 that allows for long-running tasks with feedback and status updates.

### Agent
An autonomous entity that perceives its environment and takes actions to achieve goals. In robotics, often refers to AI systems that control robot behavior.

## B

### Bridge Node
A ROS2 node that translates between different message types or protocols, often connecting AI outputs to robot commands.

## C

### Callback
A function that is executed when a specific event occurs, such as receiving a message on a topic.

### Client
A node that sends requests to a service and waits for responses.

### Collision Detection
The computational process of determining when two or more objects in a simulation or real environment intersect.

## D

### DDS (Data Distribution Service)
The middleware technology that ROS2 uses for communication between nodes, providing reliable, real-time data exchange.

### Docusaurus
A modern static website generator that enables the creation of documentation websites with features like search, versioning, and responsive design.

## E

### End Effector
The tool or device at the end of a robotic arm that interacts with the environment (e.g., gripper, camera).

### Effort Control
A type of joint control that commands torques or forces to be applied to joints.

## F

### Forward Kinematics
The process of calculating the position and orientation of the end effector given the joint angles.

### Frame
A coordinate system in 3D space, typically defined by position (x, y, z) and orientation (roll, pitch, yaw).

## G

### Gazebo
A physics-based simulation environment used in robotics for testing and validation of robot behaviors.

### Gazebo Plugin
A software component that extends Gazebo's functionality, such as providing ROS2 interfaces for simulated sensors and actuators.

## H

### Hardware Interface
The abstraction layer in ROS2 control that connects controllers to real or simulated hardware.

## I

### Inverse Kinematics
The process of calculating the required joint angles to achieve a desired end effector position and orientation.

### Inertial Properties
Physical properties of a rigid body including mass, center of mass, and moment of inertia.

## J

### Joint
A connection between two links that allows for specific types of motion (e.g., rotational, linear).

### Joint State Publisher
A ROS2 node that publishes the current state (position, velocity, effort) of all joints in a robot.

## K

### Kinematics
The study of motion without considering the forces that cause the motion, including forward and inverse kinematics.

## L

### Link
A rigid body in a robot model that has physical and visual properties.

### Launch File
An XML or Python file that defines how to start multiple ROS2 nodes with their configurations.

## M

### MoveIt!
A comprehensive motion planning framework for robotics that includes path planning, inverse kinematics, and collision detection.

### Multi-Agent System
A system composed of multiple interacting intelligent agents that work together to achieve complex tasks.

## N

### Node
A process that performs computation in the ROS2 system, capable of publishing/subscribing to topics or providing/calling services.

## P

### Publisher
A node that sends messages to a topic for other nodes to receive.

### Python Client Library (rclpy)
The Python API for ROS2 that allows Python programs to interface with ROS2 concepts like nodes, topics, and services.

## R

### Robot Operating System (ROS2)
A flexible framework for writing robot software that provides services like hardware abstraction, device drivers, and message passing.

### ROS 2 Middleware
The communication layer that enables nodes to exchange messages using DDS as the underlying transport.

### rclpy
The Python client library for ROS2, providing Python access to ROS2 functionality.

## S

### Service
A communication pattern in ROS2 that enables request-response interactions between nodes.

### Simulation
The process of creating a virtual model of a real-world system to test and validate behaviors without physical hardware.

### Subscriber
A node that receives messages from a topic published by other nodes.

## T

### Topic
A named bus over which nodes exchange messages in a publish-subscribe communication pattern.

### Transform (TF)
A system for tracking coordinate frame relationships over time in a ROS2 system.

## U

### URDF (Unified Robot Description Format)
An XML format used in ROS to describe robot models including links, joints, and other properties.

### User Story
A description of a feature from the perspective of the end user, used in agile development to capture requirements.

## V

### Velocity Control
A type of joint control that commands angular or linear velocities for joints.

### Visualization
The process of creating visual representations of robot models, sensor data, or other information in tools like RViz.

## X

### Xacro
An XML macro language that extends URDF with features like variables, constants, and macros to simplify complex robot descriptions.

## Z

### Zero Hallucination
The requirement that AI systems only provide answers based on indexed content or user-selected text, without generating information not present in the provided context.