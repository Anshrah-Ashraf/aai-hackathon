---
sidebar_position: 13
---

# Links, Joints, and Kinematics

## Understanding Links and Joints

In URDF (Unified Robot Description Format), the robot structure is defined by **links** and **joints**. Links represent the rigid bodies of the robot, while joints define the connections and allowable movements between these links. Together, they form the kinematic chain that determines how the robot can move.

## Links: The Rigid Bodies

### What are Links?

A link represents a rigid body part of the robot. It's a fundamental building block that has:
- Physical properties (mass, inertia)
- Visual representation
- Collision properties
- A coordinate frame

### Link Properties

Each link in URDF contains several important elements:

#### 1. Inertial Properties
The inertial properties define the physical characteristics of the link:

```xml
<link name="link_name">
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

- **Mass**: The mass of the link in kilograms
- **Origin**: The position and orientation of the center of mass relative to the link frame
- **Inertia**: The 3x3 inertia matrix describing how mass is distributed

#### 2. Visual Properties
Defines how the link appears in visualizations:

```xml
<link name="link_name">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
</link>
```

#### 3. Collision Properties
Defines how the link interacts in collision detection:

```xml
<link name="link_name">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>
```

## Joints: The Connections

### Joint Types

Joints define the relationship between two links and specify how they can move relative to each other:

#### 1. Fixed Joint
A fixed joint creates a rigid connection with no degrees of freedom:

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
</joint>
```

#### 2. Revolute Joint
A revolute joint allows rotation around a single axis, with limited range:

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

#### 3. Continuous Joint
A continuous joint allows unlimited rotation around a single axis:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="10" velocity="1"/>
</joint>
```

#### 4. Prismatic Joint
A prismatic joint allows linear sliding movement along an axis:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="10" velocity="1"/>
</joint>
```

#### 5. Other Joint Types
- **Planar**: Movement in a plane (2D movement)
- **Floating**: 6 degrees of freedom (rarely used)

### Joint Properties

#### 1. Origin
Defines the position and orientation of the joint relative to the parent link:

```xml
<origin xyz="0.1 0 0.2" rpy="0 0 0"/>
```

#### 2. Axis
Specifies the axis of rotation or translation:

```xml
<axis xyz="0 0 1"/>  <!-- Z-axis rotation -->
```

#### 3. Limits
For revolute and prismatic joints, define the operational limits:

```xml
<limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
```

#### 4. Dynamics
Defines friction and damping properties:

```xml
<dynamics damping="0.1" friction="0.0"/>
```

## Kinematic Chains

### Forward Kinematics

Forward kinematics calculates the position and orientation of the end effector given the joint angles. For example, in a simple 2-DOF arm:

```xml
<!-- Joint 1: Shoulder -->
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
</joint>

<!-- Joint 2: Elbow -->
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm_link"/>
  <child link="lower_arm_link"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
</joint>
```

### Denavit-Hartenberg Parameters

While not explicitly used in URDF, the joint definitions implicitly define the kinematic structure that can be used to calculate DH parameters for kinematic analysis.

## Humanoid Robot Examples

### Humanoid Leg Structure
```xml
<!-- Hip joint -->
<joint name="hip_joint" type="revolute">
  <parent link="torso_link"/>
  <child link="thigh_link"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
</joint>

<!-- Knee joint -->
<joint name="knee_joint" type="revolute">
  <parent link="thigh_link"/>
  <child link="shank_link"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="1.57" effort="50" velocity="2"/>
</joint>

<!-- Ankle joint -->
<joint name="ankle_joint" type="revolute">
  <parent link="shank_link"/>
  <child link="foot_link"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="30" velocity="1"/>
</joint>
```

### Humanoid Arm Structure
```xml
<!-- Shoulder joint -->
<joint name="shoulder_pitch" type="revolute">
  <parent link="torso_link"/>
  <child link="upper_arm_link"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="30" velocity="2"/>
</joint>

<joint name="shoulder_yaw" type="revolute">
  <parent link="upper_arm_link"/>
  <child link="forearm_link"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="20" velocity="2"/>
</joint>

<joint name="wrist_joint" type="revolute">
  <parent link="forearm_link"/>
  <child link="hand_link"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="3"/>
</joint>
```

## Kinematic Solvers

### ROS Kinematic Packages

Several ROS packages help with kinematic calculations:

#### 1. KDL (Kinematics and Dynamics Library)
- Provides forward and inverse kinematics
- Handles complex kinematic chains
- Available through `orocos_kdl` and `python_orocos_kdl`

#### 2. TRAC-IK
- Improved inverse kinematics solver
- Better handling of redundant manipulators
- More robust than traditional solvers

#### 3. MoveIt!
- Comprehensive motion planning framework
- Includes kinematic solvers
- Provides collision checking and trajectory generation

### Example Kinematic Calculation

```python
import PyKDL

# Create a simple chain
chain = PyKDL.Chain()

# Add segments (joint + link)
chain.addSegment(PyKDL.Segment(
    PyKDL.Joint(PyKDL.Joint.RotZ),
    PyKDL.Frame(PyKDL.Vector(0.1, 0, 0))
))

# Forward kinematics
fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
joint_array = PyKDL.JointArray(1)
joint_array[0] = 1.57  # 90 degrees

result_frame = PyKDL.Frame()
fk_solver.JntToCart(joint_array, result_frame)

print(f"End effector position: {result_frame.p}")
```

## Best Practices for Links and Joints

### 1. Consistent Naming
Use a clear naming convention:
- `base_link` for the robot's base
- `joint_name` for joints (e.g., `shoulder_pitch_joint`)
- `link_name` for links (e.g., `upper_arm_link`)

### 2. Proper Coordinate Frames
- Use right-hand rule for coordinate systems
- Place joint origins at physical joint centers
- Align axes with actual movement directions

### 3. Realistic Limits
- Set appropriate joint limits based on physical constraints
- Include realistic effort and velocity limits
- Consider safety margins in limit definitions

### 4. Mass Properties
- Include realistic mass and inertia values
- Use CAD software to calculate accurate values
- Verify that the center of mass is properly positioned

## Common Pitfalls

### 1. Floating Base Issues
A robot without a properly defined base link can cause issues:
```xml
<!-- Wrong: No fixed base -->
<joint name="floating_joint" type="floating">  <!-- Don't do this -->

<!-- Correct: Fixed base -->
<link name="base_link"/>  <!-- This serves as the fixed base -->
```

### 2. Kinematic Loops
URDF doesn't handle kinematic loops (closed chains) well. For such cases, consider:
- Breaking the loop with appropriate assumptions
- Using specialized packages like `drake`
- Approximating with simplified models

### 3. Unit Inconsistencies
Always use consistent units:
- Lengths: meters
- Angles: radians
- Mass: kilograms
- Time: seconds

## Summary

Links and joints form the foundation of robot kinematics in ROS:

- **Links** represent rigid bodies with physical and visual properties
- **Joints** define the connections and allowable movements between links
- **Kinematics** describes the mathematical relationships between joint positions and end-effector poses
- **Best practices** ensure accurate and efficient robot modeling

Understanding these concepts is essential for creating humanoid robot models that can be used effectively in simulation and control applications.