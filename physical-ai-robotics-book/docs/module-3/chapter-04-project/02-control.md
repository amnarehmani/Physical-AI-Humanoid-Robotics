---
id: m3-ch4-control
title: "Lesson 2: Motion Control"
sidebar_label: "Lesson 2: Control"
description: "Driving the robot with differential drive kinematics."
keywords:
  - kinematics
  - differential drive
  - controller
  - python
---

# Lesson 2: Motion Control

## The Differential Controller

We want to send `(v, w)` (Linear Velocity, Angular Velocity). The robot expects wheel speeds `(left_rad_s, right_rad_s)`.
We need a **Differential Drive Controller**.

```python
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController

controller = DifferentialController(name="simple_control", wheel_radius=0.2, wheel_base=0.5)

# In the loop:
# command: drive forward at 1.0 m/s, turn at 0.5 rad/s
action = controller.forward(command=[1.0, 0.5]) 

carter.apply_wheel_actions(action)
```

## Open Loop vs Closed Loop

This is **Open Loop**. We tell the wheels to spin. If the robot hits a wall, the wheels keep spinning.
In a real system, we would check Odometry.
`WheeledRobot` provides `get_world_pose()`.

```python
position, orientation = carter.get_world_pose()
print(f"I am at {position}")
```

## A Simple Square Path

To drive in a square:
1.  Move Forward for 200 steps.
2.  Turn 90 degrees (Rotate for 50 steps).
3.  Repeat.

This basic logic verifies that our physics and friction parameters are sane.

## End-of-Lesson Checklist

- [ ] I can instantiate a `DifferentialController`.
- [ ] I can convert `(v, w)` into wheel actions.
- [ ] I can read the robot's current position from the simulator.
- [ ] I have made the robot drive in a circle.
