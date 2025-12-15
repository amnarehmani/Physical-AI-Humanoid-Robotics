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

## Introduction

Now that our robot is in the scene, it is a static object. Physics applies gravity to it, keeping it on the floor, but it has no will. To move it, we must apply **actions** to its **dof** (Degrees of Freedom).

For the Carter robot, the DoFs are the wheel rotations. But we don't want to think in terms of "rotate left wheel at 5 rad/s". We want to think in terms of "move forward at 1 m/s". This translation is called **Kinematics**.

## Conceptual Understanding: Differential Drive Kinematics

Most wheeled service robots use a **Differential Drive** configuration: two driven wheels on a shared axis.

To move straight, both wheels spin at the same speed.
To rotate in place, they spin in opposite directions.
To curve, they spin at different speeds.

### The Unicycle Model

We abstract the robot as a unicycle that can move forward (`v`) and rotate (`w`).

```text
       ^ X (Forward)
       |
       |  v (Linear Velocity)
   [L] | [R]
    |--+--|  <-- Axis
       |
       |
       O  <-- Center of Rotation (w)

Parameters:
R = Wheel Radius (meters)
L = Wheel Base (distance between wheels in meters)
```

The equations to convert our desired command `(v, w)` into wheel speeds `(w_L, w_R)` are:

```latex
w_L = \frac{v - (L/2) \cdot w}{R}
```

```latex
w_R = \frac{v + (L/2) \cdot w}{R}
```

If `w` (angular velocity) is positive (turning left), the right wheel must spin faster than the left wheel.

## System Perspective: The Controller Class

In Isaac Sim, we don't need to write these equations manually. The `DifferentialController` class handles it.

```python
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController

# Initialize with the physical properties of the robot
# These MUST match the USD file, or the odometry will be wrong!
controller = DifferentialController(
    name="simple_control", 
    wheel_radius=0.24,  # m
    wheel_base=0.50     # m
)
```

### Humanoid Robotics Context
How does this apply to a bipedal robot?
A humanoid also receives `(v, w)` commands (e.g., from a joystick or Nav2). However, it has no wheels. Instead, it uses a **Locomotion Controller** (often an RL Policy or Model Predictive Control) that translates `(v, w)` into joint torques for the hips, knees, and ankles to execute a walking gait that results in that velocity. The input is the same; the execution is vastly more complex.

## Implementation: Driving in a Square

We will implement an **Open Loop** controller. This means we send commands based on time, without checking if we actually arrived.

### 1. The Control Loop
We need a state machine to switch between "Moving Forward" and "Turning".

```python
# Variables
iterator = 0
state = "move" # or "turn"

while simulation_app.is_running():
    world.step(render=True)
    if not world.is_playing():
        continue
        
    if iterator > 200:
        # Switch state after 200 frames
        if state == "move":
            state = "turn"
            iterator = 0
            print("Turning...")
        else:
            state = "move"
            iterator = 0
            print("Moving Forward...")
            
    # Calculate Action
    if state == "move":
        # Forward at 0.5 m/s, No rotation
        action = controller.forward(command=[0.5, 0.0])
    elif state == "turn":
        # No forward movement, Rotate at 1.0 rad/s (approx 60 deg/s)
        action = controller.forward(command=[0.0, 1.0])
        
    # Apply to Robot
    carter.apply_wheel_actions(action)
    
    iterator += 1
```

### 2. Reading Odometry (Truth)
Since this is a simulation, we can ask the "God View" where the robot actually is.

```python
# get_world_pose() returns position (x,y,z) and orientation (quaternion)
position, orientation = carter.get_world_pose()
print(f"Position: {position}")
```

## The Problem of Drift

If you run this code, you will notice the robot doesn't trace a perfect square. It might drift slightly. In the real world, wheels slip, floors are uneven, and motors are imperfect. This is why **Open Loop** control is rarely used. We need sensors to close the loop.

## End-of-Lesson Checklist

- [ ] I understand the relationship between Linear Velocity (v) and Wheel Speed.
- [ ] I can instantiate a `DifferentialController` with correct radius/base.
- [ ] I have implemented a state machine to sequence robot actions.
- [ ] I have observed "Drift" in the open-loop execution.
