---
id: m2-ch3-intro
title: "Chapter 3: Advanced Sensors"
sidebar_label: "Introduction"
description: "Simulating Cameras, Lidars, and IMUs in Gazebo."
keywords:
  - gazebo
  - sensors
  - camera
  - lidar
  - imu
---

# Chapter 3: Advanced Sensors

## Introduction

A robot without sensors is blind. In the physical world, sensors are expensive hardware components. In the Digital Twin, they are software plugins that generate data mimicking real devices.

Gazebo provides a suite of **Sensor Plugins** that attach to your URDF links. When the simulation runs, these plugins render images, cast rays, or compute accelerations, and publish standard ROS 2 messages (`sensor_msgs`).

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Attach** sensor plugins to URDF links.
2.  **Configure** sensor parameters (Resolution, FPS, Noise).
3.  **Visualize** sensor data in Rviz2 alongside the simulation.
4.  **Understand** the difference between Ideal and Noisy sensors.

## Tools & Prerequisites

*   **Gazebo**: (Classic or Ignition/Fortress). We will focus on Classic/ROS 2 integration.
*   **Rviz2**: To view the output.
*   **Xacro**: To keep our URDF clean.

## Real-World Robotics Use Cases

### 1. Obstacle Avoidance (Lidar)
A mobile robot navigates a warehouse. It uses a 2D Lidar to see pallets and walls. In simulation, we use a `ray` sensor to generate `LaserScan` messages, allowing us to test our navigation stack without crashing a real forklift.

### 2. Object Detection (Camera)
A humanoid looks for a cup. It uses an RGB-D (Depth) camera. In simulation, we use a `camera` or `depth` sensor. We can feed this synthetic image into a neural network to test if our vision model works before deploying it.

Let's start by giving our robot sight.
