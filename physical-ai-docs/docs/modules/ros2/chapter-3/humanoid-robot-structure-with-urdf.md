---
sidebar_position: 3
---

# Chapter 3: Humanoid Robot Structure with URDF

## Purpose of URDF in Humanoid Robotics

Unified Robot Description Format (URDF) is an XML-based format used to describe robots in terms of their physical structure, kinematic properties, and visual appearance. In humanoid robotics, URDF serves as the digital blueprint that defines how the robot's physical form relates to its control systems.

### Core Functions of URDF

- **Kinematic Description**: Defines the robot's joint structure and degrees of freedom
- **Visual Representation**: Specifies how the robot appears in simulation and visualization
- **Collision Properties**: Describes collision geometry for physics simulation
- **Inertial Properties**: Provides mass, center of mass, and inertia tensor for dynamics
- **Sensor Mounting Points**: Defines where sensors are attached on the robot

### Importance in Humanoid Robotics

For humanoid robots specifically, URDF is crucial because:

- It captures the complex kinematic structure of human-like bodies
- It defines the relationship between different body parts (torso, arms, legs)
- It enables proper simulation of human-like movements and interactions
- It provides the foundation for motion planning and control algorithms

## Links, Joints, and Kinematic Chains

### Links: The Building Blocks

Links represent rigid bodies in the robot structure:

```xml
<link name="base_link">
  <inertial>
    <mass value="1.0" />
    <origin xyz="0 0 0" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.1 0.1 0.1" />
    </geometry>
  </collision>
</link>
```

A link contains:
- **Inertial**: Mass properties for dynamics simulation
- **Visual**: How the link appears in visualization
- **Collision**: How the link interacts in collision detection

### Joints: Connecting the Links

Joints define the relationship between links:

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link" />
  <child link="child_link" />
  <origin xyz="0.1 0 0" rpy="0 0 0" />
  <axis xyz="0 0 1" />
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0" />
</joint>
```

Joint types include:
- **Revolute**: Rotational joint with limited range
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint
- **Fixed**: No movement between links
- **Floating**: 6-DOF movement (rarely used)
- **Planar**: Movement in a plane

### Kinematic Chains in Humanoid Robots

Humanoid robots have multiple kinematic chains:

- **Arm Chains**: From torso to hand (shoulder → elbow → wrist)
- **Leg Chains**: From torso to foot (hip → knee → ankle)
- **Spine Chain**: Torso segments from pelvis to head
- **Head Chain**: Neck joints for gaze control

These chains can be open (end-effector free) or closed (forming loops).

## How URDF Connects Software Control to Physical Bodies

### Forward Kinematics

URDF enables the calculation of end-effector positions based on joint angles:

```python
# Using a kinematics library with URDF
import kdl_parser_py.urdf as kdl_urdf
import PyKDL as kdl

# Parse URDF
(robot_model, error) = kdl_urdf.treeFromFile(urdf_file)

# Create forward kinematics solver
chain = robot_model.getChain("torso", "right_hand")
fk_solver = kdl.ChainFkSolverPos_recursive(chain)

# Calculate end-effector position
joint_angles = kdl.JntArray(chain.getNrOfJoints())
# Set joint angles based on control commands
fk_solver.JntToCart(joint_angles, end_effector_pose)
```

### Inverse Kinematics

URDF enables the calculation of required joint angles to achieve desired end-effector positions:

```python
# Inverse kinematics for humanoid control
ik_solver = kdl.ChainIkSolverPos_NR(chain, fk_solver, ik_velocity_solver)

# Calculate required joint angles for desired pose
desired_pose = kdl.Frame(kdl.Rotation.RPY(roll, pitch, yaw), kdl.Vector(x, y, z))
result = ik_solver.CartToJnt(joint_angles, desired_pose, new_joint_angles)
```

### Control Integration

URDF connects to control systems through:

1. **Joint State Interface**: Reading current joint positions
2. **Effort/Torque Interface**: Applying forces to joints
3. **Velocity Interface**: Controlling joint velocities
4. **Position Interface**: Controlling joint positions

## Preparing Humanoid Models for Simulation and Control

### URDF Best Practices

When creating humanoid URDF models:

1. **Proper Mass Distribution**: Ensure realistic inertial properties
2. **Collision Optimization**: Use simplified geometries for collision detection
3. **Visual Quality**: Use detailed meshes for visualization
4. **Joint Limits**: Set realistic ranges of motion
5. **Gazebo Integration**: Add Gazebo-specific tags if using simulation

### Example Humanoid URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5" />
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0" />
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5" />
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0" />
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02" />
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0.0 0.0 0.3" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0" />
  </joint>

  <!-- Add more links and joints for arms, legs, etc. -->
</robot>
```

## Reading and Interpreting Existing URDF Files

### Key Elements to Identify

When examining a URDF file:

1. **Root Link**: The base link from which the kinematic tree starts
2. **Joint Hierarchy**: How links are connected through joints
3. **Joint Types and Limits**: What movements are possible
4. **Physical Properties**: Mass, geometry, and inertial parameters
5. **Material Properties**: Visual appearance and collision properties

### URDF Validation

Always validate URDF files:

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Visualize the robot structure
urdf_to_graphiz /path/to/robot.urdf
```

### Common URDF Issues

- **Disconnected links**: Missing joints between expected parts
- **Inconsistent units**: Mixed metric/imperial measurements
- **Invalid joint limits**: Limits that exceed physical capabilities
- **Missing inertial properties**: Required for dynamics simulation
- **Non-physical geometries**: Impossible shapes or sizes

## Integration with Control Systems

URDF enables seamless integration with ROS 2 control systems:

- **ros2_control**: Hardware interface for real robots
- **Gazebo**: Physics simulation environment
- **MoveIt**: Motion planning framework
- **rviz2**: Visualization tools

The URDF serves as the single source of truth for robot structure, enabling all these systems to work together harmoniously and providing the foundation for sophisticated humanoid robot control.

This integration between URDF and control systems represents the crucial connection between the software representation of the robot and its physical manifestation, enabling AI systems to control humanoid robots effectively.