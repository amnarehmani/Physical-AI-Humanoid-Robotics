---
id: m2-ch2-intro
title: "Chapter 2: Physics & Dynamics"
sidebar_label: "Introduction"
description: "Adding mass, inertia, and friction to create a realistic simulation."
keywords:
  - gazebo
  - physics
  - inertia
  - friction
  - urdf
---

# Chapter 2: Physics & Dynamics

## 1. Introduction

A visual model looks like a robot, but acts like a ghost. It passes through walls, has no weight, and cannot push buttons. To create a true **Digital Twin**, we must simulate the laws of physics.

In this chapter, we will breathe life into our URDF models by adding `<inertial>` properties, collision geometries, and surface parameters like friction and damping. Without these, your control algorithms will work in simulation but fail catastrophically in the real world.

## 2. Conceptual Understanding: The Invisible Forces

Physics simulation is not about rendering; it's about solving the equation of motion.

For linear motion:
`F = ma`

For rotational motion (robots):
`tau = I * alpha`

Where:
*   `tau` is **Torque** (the muscle).
*   `I` is **Inertia** (the resistance to rotation).
*   `alpha` is **Angular Acceleration** (the movement).

If `I` is wrong, then for a given torque `tau`, the resulting movement `alpha` will be wrong. If your simulation moves faster or slower than the real robot, your PID controllers will be untuned, and your robot will shake or crash.

## 3. System Perspective: The URDF Physics Layer

In Module 1, we focused on `<visual>`. Now we introduce the "Physics Layer" tags.

```text
       [ The Robot Link Layer Cake ]

      +-----------------------------+
      |        VISUAL MESH          |
      |   (Detailed .DAE / .STL)    |
      +-----------------------------+
                  |
      +-----------------------------+
      |      COLLISION SHAPE        |
      |    (Simple Box/Cylinder)    |
      +-----------------------------+
                  |
      +-----------------------------+
      |      INERTIAL BLOCK         |
      |    (Mass, COM, Matrix)      |
      +-----------------------------+
```

1.  **Visual**: The skin. Used *only* for rendering. GPU heavy, Physics light.
2.  **Collision**: The shell. Used for contact detection. Must be simple (Primitives) to keep the simulation fast.
3.  **Inertial**: The soul. Defines Mass (kg) and distribution (`Ixx`, etc). Critical for stability.

## 4. Real-World Example: The "Exploding" Robot

A common rookie mistake is to define a small link (like a finger) but give it a huge mass (1kg) and tiny inertia (0.000001).

In Gazebo, this leads to **Numerical Instability**. The physics engine tries to divide by a near-zero number to solve the acceleration.
**Result**: The robot "explodes"—links fly off to infinity at light speed, or the robot vibrates until it clips through the floor.

**Rule of Thumb**: If your simulation explodes, check your `<inertial>` tags.

## 5. Learning Objectives

By the end of this chapter, you will be able to:

1.  **Calculate** and specify inertia tensors for robot links using primitives.
2.  **Define** collision geometries separate from visual geometries.
3.  **Tune** joint dynamics (damping and friction) to match real motors.
4.  **Debug** physics instabilities in Gazebo.

## 6. Engineering Insights: The `Sim-to-Real` Gap

If you tell a real 5kg arm to move at 10 rad/s instantly, the motors will stall or the robot will tip over. If your simulation thinks the arm weighs 0kg (or has perfect motors), it will move instantly.

This discrepancy is the `Sim-to-Real` Gap. Closing this gap starts with accurate mass and inertia properties. If you cheat here, your AI trained in simulation will be useless in reality.

Let's start by understanding Inertia.