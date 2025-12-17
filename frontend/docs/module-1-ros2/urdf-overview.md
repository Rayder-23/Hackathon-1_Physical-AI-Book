---
sidebar_position: 5
---

# URDF Overview: Unified Robot Description Format

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its kinematic structure, visual appearance, and collision properties. For humanoid robotics, URDF is essential for simulation, visualization, and motion planning.

## Purpose of URDF in Humanoid Robotics

URDF serves several critical functions in humanoid robotics:

1. **Kinematic Description**: Defines the joint structure and kinematic chain of the humanoid
2. **Visual Representation**: Specifies how the robot appears in simulation and visualization tools
3. **Collision Detection**: Provides simplified geometries for collision checking
4. **Dynamics Simulation**: Contains inertial properties needed for physics simulation
5. **Motion Planning**: Enables algorithms to understand the robot's physical constraints

## URDF Structure

A URDF file contains several main elements:

### 1. Links
Links represent rigid bodies of the robot. Each link has:
- Visual properties (shape, color, mesh)
- Collision properties (simplified geometry for collision detection)
- Inertial properties (mass, center of mass, inertia tensor)

### 2. Joints
Joints connect links and define their relative motion. Joint types include:
- **Revolute**: Rotational joint with limits
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint
- **Fixed**: No relative motion between links
- **Floating**: 6-DOF motion
- **Planar**: Motion on a plane

### 3. Materials
Define visual appearance properties like color and texture.

## Basic URDF Example

Here's a simplified example of a humanoid robot's torso and head:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## URDF in the Physical AI Context

For Physical AI applications, URDF enables:

1. **Simulation Fidelity**: Accurate representation of the robot's physical properties
2. **Embodied Intelligence**: AI systems can understand the physical constraints of the robot
3. **Safe Operation**: Collision detection prevents self-collision and environmental collisions
4. **Motion Planning**: Algorithms can generate physically feasible trajectories
5. **Reality Gap Minimization**: Better transfer from simulation to real robots

## Advanced URDF Features

### 1. Transmission Elements
Define how actuators connect to joints:

```xml
<transmission name="neck_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="neck_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="neck_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### 2. Gazebo-Specific Extensions
Additional properties for simulation:

```xml
<gazebo reference="head">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

### 3. Xacro for Complex Robots
Xacro (XML Macros) allows parameterization and reuse:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_joint" params="name parent child joint_type origin_xyz *axis *limit">
    <joint name="${name}" type="${joint_type}">
      <parent link="${parent}"/>
      <child link="${child}"/>
      <origin xyz="${origin_xyz}" rpy="0 0 0"/>
      <xacro:insert_block name="axis"/>
      <xacro:insert_block name="limit"/>
    </joint>
  </xacro:macro>
</robot>
```

## URDF Tools and Visualization

Several ROS tools work with URDF:

1. **RViz**: Visualize robot models in 3D
2. **Robot State Publisher**: Publishes joint states as transforms
3. **TF2**: Provides coordinate transforms between robot parts
4. **MoveIt!**: Uses URDF for motion planning

## Best Practices

1. **Use consistent naming conventions** for links and joints
2. **Parameterize with Xacro** for complex or variable robots
3. **Validate URDF** using `check_urdf` command
4. **Separate visual and collision geometry** appropriately
5. **Include realistic inertial properties** for accurate simulation
6. **Test in simulation** before using on real hardware

## Integration with Other ROS 2 Packages

URDF integrates with several ROS 2 packages:

- **robot_state_publisher**: Publishes transforms based on joint states
- **joint_state_publisher**: Publishes joint states for visualization
- **rviz2**: Visualizes the robot model
- **MoveIt 2**: Uses URDF for motion planning
- **Gazebo/Humble**: Uses URDF for simulation

## Common URDF Issues and Solutions

1. **Self-collision**: Ensure proper joint limits and collision geometries
2. **Inertial errors**: Use proper CAD tools to calculate accurate inertial properties
3. **Visualization problems**: Check that all links have proper visual elements
4. **Kinematic errors**: Verify joint types and limits match real hardware

URDF is fundamental to the integration of humanoid robots with ROS 2 and serves as the bridge between the physical and computational aspects of Physical AI systems. The next section will cover the learning outcomes for Module 1.