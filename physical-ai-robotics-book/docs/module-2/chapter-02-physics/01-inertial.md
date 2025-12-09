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

## The Inertial Tag

In URDF, every `<link>` that should respond to gravity needs an `<inertial>` block.

```xml
<link name="base_link">
  <visual>...</visual>
  <collision>...</collision>
  <inertial>
    <mass value="5.0"/>
    <origin xyz="0 0 0.1"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" 
             iyy="0.1" iyz="0.0" 
             izz="0.1"/>
  </inertial>
</link>
```

## What is the Inertia Tensor?

Mass is simple (kg). Inertia is "resistance to rotation."
It is a 3x3 matrix, but since it's symmetric, we define 6 numbers: `ixx`, `iyy`, `izz` (diagonal) and `ixy`, `ixz`, `iyz` (off-diagonal).

*   **High Inertia**: Hard to spin (like a flywheel).
*   **Low Inertia**: Easy to spin (like a pencil).

### Common Shapes

For a Box of width $w$, depth $d$, height $h$, mass $m$:
$$I_{xx} = \frac{1}{12}m(h^2 + d^2)$$
$$I_{yy} = \frac{1}{12}m(w^2 + h^2)$$
$$I_{zz} = \frac{1}{12}m(w^2 + d^2)$$

## Xacro Macros for Inertia

Calculating these manually is tedious. Use Xacro macros!

```xml
<xacro:macro name="box_inertia" params="m w h d">
  <inertial>
    <mass value="${m}"/>
    <inertia ixx="${(m/12) * (h*h + d*d)}" ixy="0.0" ixz="0.0"
             iyy="${(m/12) * (w*w + h*h)}" iyz="0.0"
             izz="${(m/12) * (w*w + d*d)}"/>
  </inertial>
</xacro:macro>
```

Usage:
```xml
<xacro:box_inertia m="5.0" w="0.5" h="0.2" d="0.5"/>
```

## Debugging: The Exploding Robot

If you set mass to 5kg but inertia to 0.00001 (too small), the physics engine will become unstable. The robot might vibrate violently or fly into space.
**Rule of Thumb**: If it looks like a brick, give it the inertia of a brick. Don't guess.

## End-of-Lesson Checklist

- [ ] I understand the difference between visual, collision, and inertial geometries.
- [ ] I can write an `<inertial>` block for a link.
- [ ] I have set up Xacro macros to calculate inertia for basic shapes (box, cylinder, sphere).
- [ ] I know that small inertia values can cause simulation instability.
