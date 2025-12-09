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

## Introduction

A visual model looks like a robot, but acts like a ghost. It passes through walls, has no weight, and cannot push buttons. To create a true **Digital Twin**, we must simulate the laws of physics.

In this chapter, we will breathe life into our URDF models by adding `<inertial>` properties, collision geometries, and surface parameters like friction and damping. Without these, your control algorithms will work in simulation but fail catastrophically in the real world.

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Calculate** and specify inertia tensors for robot links.
2.  **Define** collision geometries separate from visual geometries.
3.  **Tune** joint dynamics (damping and friction) to match real motors.
4.  **Debug** "exploding" simulations caused by bad physics parameters.

## Tools & Prerequisites

*   **Gazebo**: The physics simulator.
*   **MeshLab / Blender**: Useful for calculating volume and center of mass.
*   **URDF**: The file format we learned in Module 1.

## Why Physics Matters?

If you tell a real 5kg arm to move at 10 rad/s instantly, the motors will stall or the robot will tip over. If your simulation thinks the arm weighs 0kg, it will move instantly. This discrepancy is the "Sim-to-Real Gap." Closing this gap starts here.

## Real-World Robotics Use Cases

### 1. The Top-Heavy Mobile Base
A delivery robot has a heavy battery at the bottom. If you model it as a uniform box, the center of mass (COM) is too high. In simulation, it drives fine. In reality, it tips over when turning. Correctly modeling the low COM prevents this.

### 2. Gripper Friction
You are simulating a pick-and-place task. If the friction coefficient of your gripper pads is set to 0.0 (ice), the object will slide out every time. Setting it to 1.0 (rubber) allows for realistic grasping tests.

Let's start by giving our robot some mass.
