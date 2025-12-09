---
id: 00-intro
title: "Introduction to Digital Twins"
sidebar_label: "Introduction"
description: "Overview of Module 2, focusing on building a Digital Twin using Gazebo and Unity."
keywords:
  - digital twin
  - gazebo
  - unity
  - simulation
  - introduction
---

# Module 2: The Digital Twin (Gazebo & Unity)

<h2>Introduction</h2>

Welcome to Module 2. In the previous module, we built the software brain (ROS 2) and defined the body (URDF). But a brain in a jar cannot learn. It needs an environment.

Building a real robot is slow and expensive. Instead, we build a **Digital Twin**: a virtual replica so accurate that the brain can't tell the difference between the simulation and the real world.

<h3>Why Two Simulators?</h3>

We use a "Hybrid Simulation" approach:
1.  **Gazebo Fortress**: The **Physics Engine**. It calculates gravity, friction, contacts, and sensor data (LiDAR rays). It cares about accuracy, not beauty.
2.  **Unity**: The **Visualizer**. It handles photorealistic rendering, shadows, and human interaction. It makes the simulation look real to *you* and to vision-based AI.

<h3>Learning Objectives</h3>

By the end of this module, you will be able to:
1.  **Spawn** your URDF robot into a Gazebo physics world.
2.  **Equip** your robot with simulated sensors (LiDAR, Camera, IMU).
3.  **Bridge** the simulation to Unity for high-fidelity visualization.

<h3>Prerequisites</h3>

*   **Module 1 Completion**: You need the URDF and basic ROS knowledge.
*   **Hardware**: A dedicated GPU (NVIDIA GTX/RTX recommended) is helpful for 3D simulation.
*   **Software**: Gazebo Fortress and Unity 2021.3+ (LTS) installed.

<h3>Roadmap</h3>

*   **Lesson 1**: Physics. We turn XML into a falling object.
*   **Lesson 2**: Perception. We give the robot eyes.
*   **Lesson 3**: The Mirror. We connect the physics world to the game engine.

Let's build a world.