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

## 1. Introduction

In the previous chapters, we built a robot in URDF (ROS side) and simulated physics/sensors (Gazebo side). But currently, these two worlds are isolated. The ROS nodes don't know the simulation exists, and the simulation doesn't accept commands from ROS.

We need a translator. We need the **ros_gz_bridge**.

This package acts as a bi-directional gateway. It takes ROS 2 messages (e.g., `cmd_vel`), converts them into Gazebo messages, and sends them to the physics engine. Conversely, it takes Gazebo sensor data (Lidar, Cameras), converts them to ROS 2 messages, and publishes them for your nodes to see.

## 2. Conceptual Understanding: The Middleware Gap

Why can't ROS just talk to Gazebo directly?
*   **ROS 2** uses **DDS** (Data Distribution Service) as its middleware.
*   **Gazebo** uses **Ignition Transport** (Google Protobufs/ZeroMQ).

They speak different languages. The Bridge is a node that subscribes to one and publishes to the other.

```text
      [ ROS 2 Domain ]              [ Gazebo Domain ]
             |                              |
      ( /cmd_vel ) --[DDS]--> [ Bridge ] --[Ign Transport]--> ( /model/robot/cmd_vel )
             |                              |
      ( /scan )    <--[DDS]-- [ Bridge ] <--(Ign Transport)-- ( /world/demo/scan )
```

## 3. System Perspective: The Bridge Node

The bridge is just another ROS 2 executable (`ros_gz_bridge parameter_bridge`). It requires a configuration (arguments or YAML) that tells it:
1.  **Topic Name**: What topic to listen to.
2.  **Message Type**: How to convert the data (e.g., `ROS Twist` -> `Gazebo Twist`).
3.  **Direction**: One-way (`@`) or Bi-directional (`[` or `]`).

## 4. Real-World Example: Hardware-in-the-Loop (HIL)

You write a navigation stack for a real warehouse robot. To test it safely, you unplug the real motors and plug in the "Bridge."
Your code sends motor commands. The Bridge intercepts them and forwards them to the Simulator. The Simulator calculates movement and sends back fake encoder data.
**Benefit**: Your navigation code never knows the difference. You can test dangerous scenarios without risking hardware.

## 5. Learning Objectives

By the end of this chapter, you will be able to:

1.  **Configure** the `ros_gz_bridge` to map topics between ROS 2 and Gazebo.
2.  **Spawn** entities programmatically into a running simulation.
3.  **Synchronize** the simulation clock with the ROS clock (`use_sim_time`).
4.  **Debug** bridge connections using `ros2 topic list` and `gz topic -l`.

## 6. Engineering Insights: Latency

Every bridge adds latency.
*   **Zero-Copy**: Within the same process (not possible here).
*   **Network Stack**: Serialization + Deserialization takes time.

For high-speed control (e.g., balancing a drone at 500Hz), the bridge latency might be too high. In those cases, we write **Gazebo Plugins** directly in C++ (bypassing the bridge), but for general navigation, the bridge is perfect.

Let's verify our installation and start bridging.