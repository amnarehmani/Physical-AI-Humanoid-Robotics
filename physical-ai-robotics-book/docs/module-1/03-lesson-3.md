---
id: 03-lesson-3
title: "Defining the Body"
sidebar_label: "3. URDF"
description: "Introduction to URDF (Unified Robot Description Format) for defining robot structure."
keywords:
  - ros2
  - urdf
  - xml
  - robot description
  - links
  - joints
---

# Lesson 3: Defining the Body (URDF)

## 1. Introduction

We have a Nervous System (ROS 2). We have a Brain and Muscles (Python Nodes). Now, we need a Body. A robot without a body is just a theoretical ghost in the machine.

In this lesson, we will define the physical form of our humanoid using **URDF (Unified Robot Description Format)**. This matters because every simulation engine (Gazebo, Isaac Sim, Unity) and every planning algorithm (MoveIt) relies on this single XML file to understand the robot's kinematics, inertia, and geometry. If you get the URDF wrong—if you put the elbow joint in the wrong place—the robot will try to bend its arm through its own chest.

## 2. Conceptual Understanding: The Kinematic Tree

**Intuition**:
A robot is a tree.
*   **Roots**: The base (e.g., the pelvis or chassis).
*   **Branches**: The limbs (arms, legs).
*   **Leaves**: The end-effectors (hands, feet, cameras).
*   **Nodes**: The rigid parts (**Links**).
*   **Edges**: The flexible connections (**Joints**).

**Internal Mechanics**:
URDF uses XML tags to define these relationships.
1.  **`<link>`**: Defines a physical object. It has:
    *   **Visual**: What you see (meshes, colors).
    *   **Collision**: A simplified shape for physics engines to calculate bumps (saves CPU).
    *   **Inertial**: Mass and Center of Mass (CoM).
2.  **`<joint>`**: Defines the constraint between two links.
    *   **Parent**: The anchor (e.g., Torso).
    *   **Child**: The moving part (e.g., Head).
    *   **Origin**: Where the joint is located relative to the parent.
    *   **Axis**: Which way it rotates (e.g., rotate around Z-axis).

**Human Analogy**:
*   **Link**: Your femur (thigh bone).
*   **Joint**: Your hip socket (ball-and-socket).
*   **Tree**: Pelvis -> Hip Joint -> Femur -> Knee Joint -> Tibia.

## 3. System Perspective: The Tree Structure

Here is the structure of the simple humanoid we are about to build. Note how the transforms flow from the root (Base) to the leaves (Head/Arm).

```mermaid-text
       [World]
          |
     (Joint: fixed)
          v
     [Base_Link] (The Pelvis/Torso)
          |
          +---------------------+
          |                     |
   (Joint: revolute)     (Joint: fixed)
   "shoulder_joint"       "neck_joint"
          |                     |
          v                     v
     [Arm_Link]            [Head_Link]
```

## 4. Practical Example: Building "Boxy"

We will construct a robot named "Boxy" with a blue rectangular torso, a white spherical head, and a red cylindrical arm.

### The Code
The following XML defines our robot. We save this as `simple_humanoid.urdf`.

```xml title="code/module-1/simple_humanoid.urdf"
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- ========================================== -->
  <!--                LINKS                       -->
  <!-- ========================================== -->

  <!-- 1. The Torso (Root) -->
  <link name="base_link">
    <visual>
      <geometry>
        <!-- Box: x=0.3m, y=0.5m, z=0.8m -->
        <box size="0.3 0.5 0.8"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.5 0.8"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <!-- Simplified inertia tensor for a box -->
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- 2. The Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- 3. The Arm -->
  <link name="arm">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- ========================================== -->
  <!--                JOINTS                      -->
  <!-- ========================================== -->

  <!-- 1. Neck: Fixed (Head doesn't move relative to torso) -->
  <joint name="neck" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <!-- Origin: x=0, y=0, z=0.4 (Top of torso) -->
    <origin xyz="0 0 0.4"/>
  </joint>

  <!-- 2. Shoulder: Revolute (Rotates) -->
  <joint name="shoulder" type="revolute">
    <parent link="base_link"/>
    <child link="arm"/>
    <!-- Origin: Side of torso -->
    <origin xyz="0.2 0 0.3"/>
    <!-- Axis: Rotates around Y axis (Pitch) -->
    <axis xyz="0 1 0"/>
    <!-- Limits: Angles in radians -->
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

</robot>
```

## 5. Engineering Insights

When you move from "Boxy" to a real robot, things get messy.

### 5.1 Collision Mesh vs. Visual Mesh
*   **Visual**: A 100MB CAD file with every screw detailed. Looks great in Unity.
*   **Collision**: A simple box.
*   **Why?**: Physics engines check for collisions every millisecond. Checking if a million-polygon arm hits a million-polygon table kills the CPU. We use simple primitives (spheres, boxes, capsules) for physics to keep the simulation real-time.

### 5.2 Fixed vs. Continuous Joints
*   **Fixed**: Requires 0 computation. Perfect for sensors attached to the frame.
*   **Continuous/Revolute**: Requires a motor simulation, a PID controller, and state publishing.
*   **Rule**: If it doesn't move during operation, fix it.

## 6. Failure Modes

*   **The Exploding Robot**:
    *   *Symptom*: When simulation starts, the robot flies apart at light speed.
    *   *Cause*: **Collision Overlap**. If the "Head" sphere physically intersects with the "Torso" box at spawn time, the physics engine generates infinite repulsion force.
    *   *Fix*: Check your `<origin>` tags. Ensure there is a tiny gap between links.
*   **The Ghost Arm**:
    *   *Symptom*: The arm moves through tables and walls.
    *   *Cause*: Missing `<collision>` tags.
    *   *Fix*: Ensure every link has a defined collision volume, not just a visual one.

## 7. Summary

You have defined the DNA of your robot.
1.  **Links** are the bones.
2.  **Joints** are the connections.
3.  **URDF** is the file format.

This file is the single source of truth. In the next module, we will take this static XML file, load it into a physics simulator, and use our Python nodes to make "Boxy" come alive.
