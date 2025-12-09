---
id: m2-ch2-friction
title: "Lesson 2: Friction and Contact"
sidebar_label: "Lesson 2: Friction"
description: "Tuning surface properties for realistic interaction."
keywords:
  - gazebo
  - friction
  - mu1
  - mu2
  - kp
  - kd
---

# Lesson 2: Friction and Contact

## The Slip and Grip

In Gazebo, friction is defined in the `<collision>` block (standard URDF) or via `<gazebo>` extension tags.

## Standard Friction (mu)

*   `mu1`: Static friction coefficient (Coulomb friction).
*   `mu2`: Dynamic friction coefficient (often set same as mu1).

Values:
*   **0.0**: Ice (No friction).
*   **1.0**: Rubber (High friction).
*   **0.5**: Plastic on Plastic.

```xml
<gazebo reference="wheel_link">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
</gazebo>
```
For wheels, you want high friction so they don't spin in place. For a caster wheel (the passive pivot wheel), you might want low friction to let it slide (or model the swivel properly).

## Stiffness and Damping (Kp, Kd)

When two objects touch, Gazebo models the contact as a spring-damper system.
*   **Kp (Stiffness)**: How "hard" the surface is. High = Metal. Low = Sponge.
*   **Kd (Damping)**: How much energy is lost. Prevents bouncing.

If your robot jitters when standing still, try reducing `Kp` or increasing `Kd`.

## Collision Geometry vs Visual Geometry

**Pro Tip**: Never use complex high-poly meshes for collision. It kills performance.
*   **Visual**: Detailed mesh (`robot_fancy.dae`).
*   **Collision**: Simple primitive (Cylinder, Box).

```xml
<link name="wheel">
  <visual>
    <geometry><mesh filename="wheel_detailed.dae"/></geometry>
  </visual>
  <collision>
    <!-- Simple Cylinder for Physics -->
    <geometry><cylinder radius="0.1" length="0.05"/></geometry>
  </collision>
</link>
```

## End-of-Lesson Checklist

- [ ] I can adjust the friction coefficients (`mu1`, `mu2`) for a link.
- [ ] I understand why wheels need high friction.
- [ ] I am using simple geometry for collisions to save CPU cycles.
- [ ] I know how to tune contact stiffness (`kp`, `kd`) to stop jitter.
