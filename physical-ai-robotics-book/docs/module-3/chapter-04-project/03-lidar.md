---
id: m3-ch4-lidar
title: "Lesson 3: Lidar Obstacle Avoidance"
sidebar_label: "Lesson 3: Lidar"
description: "Reading RTX Lidar data and reacting."
keywords:
  - lidar
  - rtx
  - obstacle avoidance
  - python
---

# Lesson 3: Lidar Obstacle Avoidance

## Introduction

In the previous lesson, our robot moved blindly. If we placed a cube in its path, it would crash. To make the robot intelligent, we must close the loop:

1.  **Sense**: Read the environment.
2.  **Plan**: Decide what to do.
3.  **Act**: Move the motors.

This lesson introduces the **Lidar** (Light Detection and Ranging) sensor and how to process its data using NumPy.

## Conceptual Understanding: RTX Lidar

How do we simulate a laser scanner?
1.  **Rasterization (Standard)**: The engine renders a depth map from the sensor's point of view. It is fast but inaccurate. It cannot see reflections (mirrors) or transparency (glass).
2.  **Ray Tracing (RTX)**: The engine shoots thousands of photons (rays) into the scene. They bounce off surfaces based on physical material properties.

For modern robotics, **RTX** is essential. If your robot is patrolling an office with glass doors, a raster-based Lidar might think the door is open (because glass is transparent), leading to a crash. An RTX Lidar will detect the reflection or refraction.

### The Sense-Plan-Act Loop

```text
       [ World ]
           |
      (Photons)
           |
           v
      [ Lidar Sensor ]
           |
      (Float Array)
           |
           v
+--------------------------+
|   PYTHON SCRIPT          |
|  1. Slice Array          |  <-- "Is there an object in front?"
|  2. Heuristic Logic      |  <-- "If yes, Turn Left"
|  3. Differential Ctrl    |  <-- "Set w = 1.0"
+--------------------------+
           |
      (Wheel Cmd)
           |
           v
      [ Robot ]
```

## Implementation: The Braitenberg Vehicle

A "Braitenberg Vehicle" is a thought experiment where simple sensor-motor connections create complex-looking behavior. We will implement a "Fear" behavior: if the robot sees an object, it turns away.

### 1. Acquiring the Interface
The sensor data is not available on the robot object directly. We must ask the specialized `_range_sensor` interface.

```python
from omni.isaac.range_sensor import _range_sensor

# Acquire the singleton interface
lidar_interface = _range_sensor.acquire_range_sensor_interface()

# Define where the sensor is in the USD hierarchy
# Note: The Carter robot usually comes with a Lidar at this path
lidar_path = "/World/Carter/chassis_link/lidar"
```

### 2. Processing the Data
The Lidar returns a linear array of depth values (e.g., 360 values for 360 degrees). We use **NumPy** to check specific sectors.

```python
import numpy as np

# Inside the simulation loop...
depth = lidar_interface.get_depth_data(lidar_path)
    
# Example: 360 degree scan. 
# Index 0 is -180 deg, Index 180 is Center (0 deg), Index 360 is +180 deg.
# Let's check the center 20 degrees (Indices 170 to 190)
center_scan = depth[170:190]

# Find the closest object in this sector
min_dist = np.min(center_scan)

print(f"Distance to obstacle: {min_dist:.2f} m")
```

### 3. The Logic (The "Brain")
Now we write the behavior.

```python
# Threshold: 1.0 meter
if min_dist < 1.0:
    # FEAR: Obstacle detected! 
    # Stop moving forward (v=0) and Rotate (w=1.0)
    print("Obstacle! Turning...")
    action = controller.forward(command=[0.0, 1.0])
else:
    # CRUISE: Path clear.
    # Move forward (v=1.0) and go straight (w=0.0)
    action = controller.forward(command=[1.0, 0.0])

carter.apply_wheel_actions(action)
```

## Humanoid Robotics Context

Humanoid robots (like Tesla Optimus or Figure 01) rarely use 2D Lidars. They rely on **Vision** (RGB-D Cameras). However, the logic remains identical:
1.  Input: Depth Image (Matrix) instead of Depth Array (Vector).
2.  Process: "Is the center of the image close?"
3.  Action: Stop walking.

In Module 4, we will replace this simple `if/else` logic with a Neural Network (VLA) that can understand *what* it is seeing ("That is a human, stop"), not just *that* it is seeing something.

## End-of-Lesson Checklist

- [ ] I can explain the difference between Raster and RTX Lidar.
- [ ] I can access the Lidar data array using the python interface.
- [ ] I have implemented sector slicing using NumPy.
- [ ] I have successfully built a robot that patrols without crashing.
