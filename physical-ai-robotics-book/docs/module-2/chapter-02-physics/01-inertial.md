---
id: m2-ch2-inertial
title: "Lesson 1: Mass and Inertia"
sidebar_label: "Lesson 1: Inertia"
description: "Defining mass and the inertia tensor in URDF."
keywords:
  - urdf
  - inertia
  - mass
  - ixx
  - iyy
---

# Lesson 1: Mass and Inertia

## 1. Introduction

When you hold a sledgehammer, it feels heavy. But it feels *different* depending on whether you hold it by the handle or by the head.
*   **Mass** is the same (5kg).
*   **Inertia** changes depending on where the mass is distributed relative to the rotation axis.

In robotics, motors don't just "lift weight." They "spin inertia." If you tell Gazebo a link weighs 5kg but has zero inertia, Gazebo sees a mathematical paradox and the simulation will crash.

In this lesson, we will learn how to correctly define the `<inertial>` tag in URDF.

## 2. Conceptual Understanding: The Tensor

We represent rotational inertia not as a single number, but as a **3x3 Symmetric Matrix** called the Inertia Tensor.

```text
       [  Ixx   Ixy   Ixz  ]
  I =  [  Ixy   Iyy   Iyz  ]
       [  Ixz   Iyz   Izz  ]

  Diagonal Terms (Ixx, Iyy, Izz):
  -------------------------------
      ^ Z         ^ Z         ^ Z
      |           |           |
  X <-O-> X   Y <-O-> Y   Z <-O-> Z
    (Spin X)    (Spin Y)    (Spin Z)
```

*   **Diagonal Terms**: The resistance to spinning around the principal axes (`X`, `Y`, and `Z`).
*   **Off-Diagonal Terms (`Ixy`, etc.)**: Describe imbalance (like a wobbling fan). For symmetric shapes aligned with the axes, these are **zero**.

### Intuition
*   **High `Izz`**: Hard to spin around the vertical axis (like a merry-go-round).
*   **Low `Izz`**: Easy to spin (like a pencil twirling).

## 3. System Perspective: The URDF Implementation

In URDF, every dynamic `<link>` must have an `<inertial>` block.

```xml
<link name="forearm_link">
  <visual> ... </visual>
  <collision> ... </collision>

  <inertial>
    <!-- 1. Mass in Kilograms -->
    <mass value="2.5"/>
    
    <!-- 2. Center of Mass relative to link origin -->
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    
    <!-- 3. The Tensor (Only 6 numbers needed due to symmetry) -->
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" 
             iyy="0.01" iyz="0.0" 
             izz="0.005"/>
  </inertial>
</link>
```

**Critical Warning**: If you define a `<link>` without an `<inertial>` block, Gazebo treats it as a static object (glued to the world). If you want it to move, it *must* have inertia.

## 4. Implementation: Xacro Macros

Calculating integrals for `Ixx` by hand is painful. We use Xacro macros to automate this for primitive shapes.

### 4.1 The Box Macro
For a box of mass `m`, width `w` (y), depth `d` (x), height `h` (z):
`Ixx = (1/12) * m * (h^2 + w^2)`

```xml title="code/module-2/urdf/inertial_macros.xacro"
<xacro:macro name="box_inertia" params="m w h d">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${(m/12) * (h*h + d*d)}" ixy="0.0" ixz="0.0"
             iyy="${(m/12) * (w*w + h*h)}" iyz="0.0"
             izz="${(m/12) * (w*w + d*d)}"/>
  </inertial>
</xacro:macro>
```

### 4.2 The Cylinder Macro
For a cylinder of mass `m`, radius `r`, length `h` (aligned along Z):
`Izz = (1/2) * m * r^2`

```xml
<xacro:macro name="cylinder_inertia" params="m r h">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${(m/12) * (3*r*r + h*h)}" ixy="0.0" ixz="0.0"
             iyy="${(m/12) * (3*r*r + h*h)}" iyz="0.0"
             izz="${(m/2) * (r*r)}"/>
  </inertial>
</xacro:macro>
```

## 5. Real-World Example: The "Wobbling" Drone

A student team built a quadcopter simulation. They estimated the inertia as a perfect sphere. In reality, a drone has much higher inertia on the Yaw axis (arms sticking out) than the Roll axis.
**Result**: Their PID controller for Yaw was undertuned. In simulation (sphere), it was stable. In real flight, the drone spun out of control because the motors couldn't overcome the *real* inertia fast enough.
**Lesson**: Approximate your shape correctly. A drone is a Box or a Cylinder, not a Sphere.

## 6. Engineering Insights: Debugging Instability

If your simulation "explodes" (robots flying into space):

1.  **Check for Small Inertia**: Did you put `ixx="0.000001"`? Physics engines hate this. Increase it to at least `0.001` or sum the mass of small parts into the parent link.
2.  **Check Mass Ratios**: Do you have a `0.01kg` finger attached to a `100kg` arm? The "Mass Ratio" creates numerical errors (matrix ill-conditioning). Try to keep ratios within `1:1000`.
3.  **Check Center of Mass**: Is `origin xyz` outside the visual geometry? This makes the robot act like a phantom pendulum.

## 7. Summary

Inertia is the bridge between geometry and dynamics.
*   **Mass** resists linear force (`F=ma`).
*   **Inertia** resists rotational torque (`tau=I*alpha`).

By using Xacro macros, we can assign "good enough" physics properties to our robot components instantly. In the next lesson, we will look at how these objects interact with the world through **Friction and Contact**.