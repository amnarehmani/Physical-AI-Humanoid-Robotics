---
id: 02-lesson-2
title: "Simulating Perception"
sidebar_label: "2. Sensors"
description: "Adding LiDAR, Camera, and IMU sensors to Gazebo simulation."
keywords:
  - sensors
  - lidar
  - camera
  - imu
  - plugins
  - gazebo
---

# Lesson 2: Simulating Perception (Sensors)

## 1. Introduction

A robot without sensors is a hallucination. It moves assuming the world is exactly as it predicts, but the moment it slips or hits an obstacle, that prediction fails. To close the loop, we need **Perception**.

In the previous lesson, we built the "Physics Playground" where our robot exists. Now, we will give it senses:

1. **LiDAR**: To feel the geometry of the room.  
2. **Camera**: To see the texture of objects.  
3. **IMU**: To feel gravity and acceleration.

Simulating sensors is computationally expensive. It requires the computer to calculate millions of light interactions per second. In this lesson, we will learn how to add these sensors efficiently using **Gazebo Plugins** and bridge their data back to ROS 2.

---

## 2. Conceptual Understanding: Synthetic Senses

How do we simulate a sensor? We don't simulate individual photons (that would take years to render one frame). Instead, we use mathematical approximations.

### 2.1 LiDAR (Raycasting)

A real LiDAR spins a laser and measures the time-of-flight for reflections.  
A simulated LiDAR uses **Raycasting**. The physics engine shoots a mathematical line from the sensor origin and calculates the intersection point with the nearest collision mesh.

`Distance = || Point_hit - Point_origin ||`

This approach is fast but unrealistically perfect. Real LiDARs suffer from noise, surface absorption, and reflection errors on glass or dark materials. Gazebo allows us to inject **Gaussian Noise** to better approximate real-world behavior.

---

### 2.2 Cameras (Rasterization)

A simulated camera is essentially a 3D render. Gazebo positions a virtual camera at the sensor link’s frame, renders the scene to an off-screen buffer, and copies that pixel buffer into a ROS message.

**Note**: This is the most CPU/GPU-intensive sensor.  
Higher resolution directly reduces frame rate and increases latency.

---

### 2.3 IMU (Inertial Measurement Unit)

The IMU is unique because it does not observe the external world; it measures the robot’s internal motion state.

- **Accelerometer**: Measures linear acceleration (including gravity)
- **Gyroscope**: Measures angular velocity

Gazebo computes IMU values directly from the physics state of the link to which the IMU is attached.

---

## 3. System Perspective: The Plugin Architecture

Gazebo runs as a separate process from ROS 2 and does not natively understand ROS concepts like topics or messages. To bridge this gap, we use **Plugins**.

A plugin is a C++ library attached to a simulated model.

1. **Sensor Plugin**: Generates raw data (e.g., `gpu_lidar`)
2. **ROS Bridge**: `ros_gz_bridge` transfers messages between Gazebo and ROS 2

```text
+---------------------+       +---------------------------+       +---------------------+
|      ROS 2          |       |      ROS_GZ_BRIDGE        |       |   Gazebo Fortress   |
|                     |       |                           |       |                     |
|  [Nav Stack]        |       |                           |       |  [Robot Model]      |
|    Sub: /scan       +<------+ /scan <--------- /scan    +<------+    [Lidar Plugin]   |
|                     |       |                           |       |      (Raycast)      |
|  [Computer Vision]  |       |                           |       |                     |
|    Sub: /image_raw  +<------+ /image <-------- /image   +<------+    [Camera Plugin]  |
|                     |       |                           |       |      (Render)       |
|  [EKF Filter]       |       |                           |       |                     |
|    Sub: /imu        +<------+ /imu <---------- /imu     +<------+    [IMU Plugin]     |
+---------------------+       +---------------------------+       +---------------------+
