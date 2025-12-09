---
id: m2-ch4-intro
title: "Chapter 4: The ROS 2 - Gazebo Bridge"
sidebar_label: "Introduction"
description: "Connecting the Operating System to the Simulator."
keywords:
  - ros_gz_bridge
  - gazebo
  - ros2
  - simulation
  - middleware
---

# Chapter 4: The ROS 2 - Gazebo Bridge

## Introduction

In the previous chapters, we built a robot in URDF (ROS side) and simulated physics/sensors (Gazebo side). But currently, these two worlds are isolated. The ROS nodes don't know the simulation exists, and the simulation doesn't accept commands from ROS.

We need a translator. We need the **ros_gz_bridge**.

This package acts as a bi-directional gateway. It takes ROS 2 messages (e.g., `cmd_vel`), converts them into Gazebo messages, and sends them to the physics engine. Conversely, it takes Gazebo sensor data (Lidar, Cameras), converts them to ROS 2 messages, and publishes them for your nodes to see.

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Configure** the `ros_gz_bridge` to map topics between ROS 2 and Gazebo.
2.  **Spawn** entities programmatically into a running simulation.
3.  **Synchronize** the simulation clock with the ROS clock (`use_sim_time`).
4.  **Debug** bridge connections using `ros2 topic list` and `gz topic -l`.

## Tools & Prerequisites

*   **ros_gz**: The official suite of packages (`sudo apt install ros-humble-ros-gz`).
*   **Understanding of Topics**: You must know what `std_msgs` and `geometry_msgs` are.

## Real-World Robotics Use Cases

### 1. Hardware-in-the-Loop (HIL)
You write a navigation stack for a real warehouse robot. To test it safely, you unplug the real motors and plug in the "Bridge." Your code sends motor commands, the Bridge forwards them to the Simulator, the Simulator calculates movement and sends back fake encoder data. Your code never knows the difference.

### 2. Cloud Simulation
You run the physics engine on a powerful cloud server (AWS RoboMaker) while your ROS 2 control logic runs on a local laptop. The Bridge handles the communication over the network, allowing massive scale testing.

Let's verify our installation and start bridging.
