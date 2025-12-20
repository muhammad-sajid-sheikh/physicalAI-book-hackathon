---
sidebar_position: 4
title: 'Chapter 1.3 — URDF & Humanoid Modeling'
---

# Chapter 1.3: URDF & Humanoid Modeling

## Introduction

The Unified Robot Description Format (URDF) is the standard way to describe robots in ROS. It provides a complete kinematic and dynamic description of a robot, including its physical properties, joints, and visual elements. For humanoid robotics, URDF is essential for creating accurate models that can be used in simulation and real-world applications.

## Learning Objectives

By the end of this chapter, you should be able to:
1. Understand the structure and components of URDF files
2. Create basic robot models using URDF
3. Define joints, links, and materials in URDF
4. Model humanoid robots with appropriate joint configurations
5. Validate and visualize URDF models

## What is URDF?

URDF (Unified Robot Description Format) is an XML-based format used to describe robots in ROS. It contains information about:

- **Links**: Rigid bodies with physical properties
- **Joints**: Connections between links with kinematic properties
- **Visual elements**: How the robot appears in simulation
- **Collision elements**: How the robot interacts with the environment
- **Inertial properties**: Mass, center of mass, and inertia tensor

## URDF Structure

A basic URDF file follows this structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links definition -->
  <link name="link_name">
    <inertial>
      <!-- Mass and inertial properties -->
    </inertial>
    <visual>
      <!-- Visual representation -->
    </visual>
    <collision>
      <!-- Collision representation -->
    </collision>
  </link>

  <!-- Joints definition -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
  </joint>
</robot>
```

## Links

Links represent rigid bodies in the robot. Each link can have:

### Inertial Properties
```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
</inertial>
```

### Visual Properties
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- or <cylinder radius="1" length="1"/> -->
    <!-- or <sphere radius="1"/> -->
    <!-- or <mesh filename="package://path/to/mesh.stl"/> -->
  </geometry>
  <material name="color_name">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
</visual>
```

### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

## Joints

Joints connect links and define their movement. URDF supports several joint types:

### Fixed Joint
```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### Revolute Joint
```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

### Continuous Joint
```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Prismatic Joint
```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="1" effort="100" velocity="1"/>
</joint>
```

## Creating a Simple Robot

Here's an example of a simple robot with a base and one moving joint:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Arm link -->
  <link name="arm_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
      <material name="red">
        <color rgba="1.0 0.0 0.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <cylinder radius="0.05" length="0.5"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## Humanoid Robot Modeling

Humanoid robots have a specific structure that typically includes:

### Body Parts
- Torso/Body
- Head
- Arms (Upper arm, Forearm, Hand)
- Legs (Upper leg, Lower leg, Foot)
- Links connected by appropriate joints

### Joint Configuration
Humanoid robots typically have these joint types:
- **6 DOF** at the torso for full body movement
- **3 DOF** per arm for shoulder (ball joint)
- **1 DOF** per elbow for flexion
- **2 DOF** per wrist for rotation and flexion
- **3 DOF** per leg for hip (ball joint)
- **1 DOF** per knee for flexion
- **2 DOF** per ankle for movement

### Example Humanoid Structure
```xml
<robot name="humanoid_robot">
  <!-- Torso -->
  <link name="torso">
    <!-- Torso properties -->
  </link>

  <!-- Head -->
  <link name="head">
    <!-- Head properties -->
  </link>

  <joint name="torso_to_head" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <!-- Left upper arm properties -->
  </link>

  <joint name="torso_to_left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  <!-- Additional arm joints would follow -->
</robot>
```

## URDF Tools and Validation

### Checking URDF Files
You can validate URDF files using the `check_urdf` command:
```bash
check_urdf /path/to/robot.urdf
```

### Visualizing URDF
You can visualize URDF files in RViz or using the `joint_state_publisher_gui`:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(cat robot.urdf)
```

## Xacro: XML Macros for URDF

Xacro is a macro language that simplifies complex URDF files:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_cylinder" params="name radius length xyz rpy">
    <link name="${name}">
      <inertial>
        <mass value="1.0"/>
        <origin xyz="${xyz}"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
      </inertial>
      <visual>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="${xyz}" rpy="${rpy}"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <xacro:simple_cylinder name="base" radius="0.2" length="0.1" xyz="0 0 0" rpy="0 0 0"/>
</robot>
```

## Best Practices for URDF

1. **Use consistent naming**: Follow a clear naming convention for links and joints
2. **Define proper inertial properties**: Accurate inertial properties are crucial for simulation
3. **Separate visual and collision models**: Use simple collision models for performance
4. **Use appropriate joint limits**: Set realistic limits based on physical constraints
5. **Validate your URDF**: Always check your URDF files for errors
6. **Use Xacro for complex robots**: Simplify complex URDFs with macros

## Exercises

1. **Simple Robot Model**: Create a URDF model of a simple robot with a base, two links, and one revolute joint. Validate and visualize it using ROS tools.

2. **Humanoid Arm**: Design a simplified humanoid arm with shoulder, elbow, and wrist joints. Include proper joint limits and visual elements.

3. **URDF Optimization**: Take an existing URDF file and convert it to Xacro format, using macros to reduce redundancy and improve maintainability.

## References

1. ROS Documentation. (2023). "URDF Tutorials". http://wiki.ros.org/urdf/Tutorials
2. ROS Documentation. (2023). "Xacro". http://wiki.ros.org/xacro
3. Chitta, S., et al. (2012). "The autonomous robotics software distribution". IEEE Robotics & Automation Magazine, 19(3), 69-79.

## Summary

URDF is a fundamental tool in ROS for describing robots. Understanding how to properly create and structure URDF files is essential for humanoid robotics, as it enables accurate simulation, visualization, and control of complex robotic systems. By mastering URDF and its companion Xacro, you can create detailed models of humanoid robots that accurately represent their physical properties and kinematic structure.