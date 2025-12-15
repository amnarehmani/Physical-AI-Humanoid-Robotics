---
id: m2-ch5-intro
title: "Chapter 5: Project - Warehouse Digital Twin"
sidebar_label: "Introduction"
description: "Building a complex multi-robot simulation environment."
keywords:
  - warehouse
  - digital twin
  - sdf
  - multi-robot
  - project
---

# Chapter 5: Project - Warehouse Digital Twin

## 1. Introduction

We have a robot. We have a bridge. We have sensors. Now we need a job.
The "Hello World" of industrial robotics is the Warehouse. It is a controlled environment, but it is dynamic, cluttered, and full of constraints.

In this capstone project for Module 2, we will build a **Warehouse Digital Twin**. We will construct a custom environment using SDF (Simulation Description Format), spawn multiple robots, and simulate a "Traffic Jam" scenario. This project consolidates everything you have learned about Physics, Sensors, and the Bridge.

## 2. Conceptual Understanding: The Environment Layer

A Digital Twin isn't just a robot; it's the robot *in context*.
*   **Static Layer**: Walls, pillars, floor. (Baked into the SDF world).
*   **Semi-Static Layer**: Shelves, pallets. (Movable but heavy).
*   **Dynamic Layer**: Boxes, other robots, humans. (Fast moving).

We must model these layers efficiently. Static objects should be simple meshes. Dynamic objects need precise physics.

## 3. System Perspective: The Simulation Stack

```text
      [ Launch File (main.launch.py) ]
               |
      +--------+--------+
      |                 |
 [ Gazebo Server ]   [ ROS 2 Stack ]
      |                 |
   (World.sdf)       (Nav2 / AMCL)
      |                 |
   [ Robot 1 ] <====> [ Namespace: /bot1 ]
   [ Robot 2 ] <====> [ Namespace: /bot2 ]
   [ Actor   ]        [ No Control ]
```

## 4. Real-World Example: Fleet Management

Amazon and Ocado use simulators with 100+ robots to test **Traffic Managers**.
*   What happens if two robots meet in a narrow aisle?
*   What if a robot breaks down and blocks the main highway?
*   What if the Wi-Fi fails in Zone B?

Simulating this at scale requires strict namespacing (e.g., `/robot_1/cmd_vel`, `/robot_2/cmd_vel`). If you mess up the namespaces, Robot 1's camera might show up on Robot 2's screen.

## 5. Learning Objectives

By the end of this chapter, you will be able to:

1.  **Design** a custom warehouse world using SDF and Gazebo Fuel assets.
2.  **Launch** multiple robots with unique namespaces (`/bot1`, `/bot2`) to prevent topic collision.
3.  **Simulate** dynamic events (falling boxes, walking humans).
4.  **Package** the entire simulation into a single command: `ros2 launch my_pkg warehouse.launch.py`.

## 6. Engineering Insights: Asset Management

Where do we get the 3D models for shelves and pallets?
*   **Gazebo Fuel**: An online repository of open-source models (like GitHub for 3D).
*   **Blender**: Create your own (export as DAE/OBJ).

**Warning**: Fuel models can be heavy. A high-poly forklift might look cool but will kill your Real-Time Factor. Always check the triangle count.

Let's start by architecting the world.