---
id: m2-ch5-fault-injection
title: "Lesson 3: Fault Injection"
sidebar_label: "Lesson 3: Fault Injection"
description: "Simulating failures to test robustness."
keywords:
  - fault injection
  - chaos engineering
  - simulation
  - testing
---

# Lesson 3: Fault Injection

## 1. Introduction

"It works on my machine."
This phrase kills robotics startups. A robot that only works in perfect conditions is useless.
To build robust **Physical AI**, we must practice **Chaos Engineering**. We intentionally break things in the simulation to see if the AI survives.

## 2. Conceptual Understanding: The Failure Matrix

We categorize failures into three types:

1.  **Sensor Failure**: The Lidar goes black. The IMU drifts to infinity.
2.  **Actuator Failure**: A motor stalls. A wheel slips.
3.  **Environmental Failure**: The lights turn off. A shelf falls over.

In a Digital Twin, we can trigger these events programmatically.

## 3. System Perspective: The Chaos Node

We create a dedicated ROS node: `chaos_monkey`.

```text
      [ Chaos Node ]
           |
    (Timer Trigger)
           |
           v
    [ Select Victim ] --> (Robot Beta)
           |
           v
    [ Select Fault ] --> (Blind Camera)
           |
           v
    [ Execute ] --> (Spawn box in front of lens)
```

## 4. Implementation: Triggering Faults

### 4.1 Blindness (Sensor)
We can "blind" a robot by spawning a small black box rigidly attached to its camera link.
`create -name lens_cap -parent robot_beta/camera_link ...`

### 4.2 Slippage (Actuator)
We can simulate an oil spill by changing the friction of the floor dynamically using Gazebo services.

### 4.3 The Kidnapped Robot (Environment)
We can teleport the robot to a random location. This breaks the AMCL localization (the robot thinks it's at A, but it's at B).
Does your navigation stack realize the scan doesn't match the map? Does it trigger global relocalization? Or does it crash into a wall?

```python
# python code to teleport robot
msg = Pose()
msg.position.x = 10.0 # Teleport far away
publisher.publish(msg)
```

## 5. Engineering Insights: Regression Testing

When you fix a bug, add a **Regression Test** in simulation.
*   *Bug*: Robot crashed when a human walked in front of it.
*   *Test*: Create a launch file that spawns the robot and a moving actor on a collision course. Run this test before every code release (CI/CD).

## 6. Summary

A simulator is not just for checking if code works. It is for checking what happens when things **don't** work.
By injecting faults, we move from "Code Verification" to "System Validation."

In the final summary, we will wrap up the entire module and prepare to hand over the robot to the Navigation team.