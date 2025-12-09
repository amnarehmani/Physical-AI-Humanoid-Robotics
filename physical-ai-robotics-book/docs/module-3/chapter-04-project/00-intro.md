---
id: m3-ch4-intro
title: "Chapter 4: Project - Isaac Sim Navigation"
sidebar_label: "Introduction"
description: "Controlling a robot in Isaac Sim via Python."
keywords:
  - isaac sim
  - python
  - navigation
  - project
---

# Chapter 4: Project - Isaac Sim Navigation

## Introduction

We have learned USD and Replicator. Now let's make a robot move.
Unlike Module 1 (ROS 2), here we will control the robot directly using the **Isaac Sim Core API**. This is useful for Reinforcement Learning (RL) where we need direct, high-speed access to the physics engine, bypassing the ROS bridge latency.

In this capstone project, we will build a **Warehouse Guard**. It will patrol a USD warehouse, checking for intruders (cubes), and logging their positions.

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Load** a robot (Carter) and an environment (Warehouse) via Python.
2.  **Control** the robot using the `DifferentialController` class.
3.  **Simulate** Lidar sensors and read data directly in Python.
4.  **Implement** a simple obstacle avoidance logic.

## Tools & Prerequisites

*   **Isaac Sim Python Environment**: `kit/python/bin/python3`.
*   **OmniGraph**: For connecting action graphs (optional but powerful).

## Architecture

We are not using ROS here.
**Driver Script (Python)** -> **Isaac Core API** -> **PhysX Engine**.
This loop runs synchronously. We step physics, read sensor, compute action, apply action. Repeat.

## Real-World Robotics Use Cases

### 1. Reinforcement Learning (RL)
To train a robot to walk, you need millions of steps. Doing this via ROS 2 is too slow. Doing it via direct API (or Isaac Gym) is 1000x faster.

### 2. System Identification
You want to calibrate the motor friction. You run a script that spins the simulated wheels at various torques and measures the speed, fitting a curve to match the real robot data.

Let's load our agent.
