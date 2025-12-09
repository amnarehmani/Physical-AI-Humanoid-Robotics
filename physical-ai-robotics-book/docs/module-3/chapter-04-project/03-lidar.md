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

## Adding a Lidar

We can add a Lidar Prim to the robot using Python commands, or use the pre-equipped Carter robot.
Let's assume the Lidar is at `/World/Carter/chassis_link/lidar`.

## Reading Data

Isaac Sim provides a helper to read sensor data.

```python
from omni.isaac.range_sensor import _range_sensor

lidar_interface = _range_sensor.acquire_range_sensor_interface()
lidar_path = "/World/Carter/chassis_link/lidar"

# In the loop:
depth = lidar_interface.get_depth_data(lidar_path)
# depth is a numpy array
```

## Simple Avoidance Logic

We check the center pixels of the depth array.

```python
import numpy as np

# Get center 10 degrees
center_scan = depth[300:400] 
min_dist = np.min(center_scan)

if min_dist < 1.0:
    # Too close! Stop and turn.
    action = controller.forward(command=[0.0, 1.0])
else:
    # Clear. Move forward.
    action = controller.forward(command=[1.0, 0.0])

carter.apply_wheel_actions(action)
```

## RTX Sensors

Standard sensors use the rasterizer (fast but low fidelity). **RTX Sensors** use Ray Tracing (slower but physically accurate). For Lidar, RTX is preferred as it correctly simulates glass, mirrors, and multipath interference.

## End-of-Lesson Checklist

- [ ] I can access the Lidar interface in Python.
- [ ] I can read the depth array as a NumPy matrix.
- [ ] I have implemented a "Braitenberg Vehicle" logic (fear of obstacles).
- [ ] The robot successfully navigates without hitting walls.
