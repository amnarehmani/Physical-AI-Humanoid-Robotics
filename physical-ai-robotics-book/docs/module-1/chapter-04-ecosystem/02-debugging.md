---
id: m1-ch4-debugging
title: "Lesson 2: Debugging & Diagnostics"
sidebar_label: "Lesson 2: Debugging"
description: "Using ros2 doctor, rqt, and logs."
keywords:
  - ros2
  - debugging
  - rqt
  - doctor
  - logging
---

# Lesson 2: Debugging & Diagnostics

## 1. Introduction

A robot that doesn't work is a paperweight. A robot that doesn't work *and you don't know why* is a nightmare.

Debugging robotics is harder than debugging web apps. You have distributed processes, real-time constraints, and physical hardware faults. You cannot just "Step Through" code when the robot is balancing on one leg.

This lesson introduces the **Diagnostic Toolkit**:
1.  **ROS 2 Doctor**: The system checkup.
2.  **RQt**: The graphical inspector.
3.  **Logging**: The voice of the code.

## 2. Conceptual Understanding: Observability

**Intuition**:
*   **Black Box**: A system you can't see inside. You put inputs in, garbage comes out. You are blind.
*   **Glass Box**: A system with transparent walls. You can see the gears turning.
*   **Observability**: The degree to which you can understand the internal state of the system just by looking at its outputs (logs, metrics, graphs).

Our goal is to turn the robot from a Black Box into a Glass Box.

## 3. System Perspective: The Introspection Layer

ROS 2 has a built-in "Introspection Layer" running parallel to your application.

```mermaid-text
[Application Layer]
   Node A ---> Topic X ---> Node B

[Introspection Layer]
   (1) ros2 doctor: Checks OS/Network health
   (2) rqt_graph: Visualizes the A->X->B connection
   (3) /rosout: Aggregates logs from A and B
```

## 4. Practical Example: The Toolkit

### 4.1 The Checkup (Doctor)
Before you debug your code, debug your system.
```bash
ros2 doctor
```
**Checks**:
*   Is the disk full?
*   Are different nodes using incompatible middleware (RMW)?
*   Is multicast blocked by the firewall?

### 4.2 The X-Ray (RQt Graph)
"Why is my robot not moving?"
Run `rqt_graph`.
*   You see: `/joystick_node` -> `/cmd_vel`
*   You see: `/motor_driver` subscribing to `/motor_cmd`
*   **Diagnosis**: The lines don't connect! You need a node to translate `/cmd_vel` to `/motor_cmd`.

### 4.3 The Heartbeat (RQt Plot)
"The motor is vibrating."
Run `rqt_plot`.
Add topic `/joint_states/velocity[0]`.
*   You see a smooth sine wave? Good.
*   You see jagged noise? Bad. Tune your PID D-gain.

## 5. Engineering Insights: Logging Strategy

`print()` is forbidden in production code. Use the logger.

*   **Throttle**: Don't flood the console at 1000Hz.
    ```python
    # Log 'Sensor Active' only once per second
    self.get_logger().info('Sensor Active', throttle_duration_sec=1.0)
    ```
*   **Skip First**: Ignore the noise during startup.
    ```python
    self.get_logger().warn('Low Battery', skip_first=True)
    ```
*   **Levels**:
    *   `DEBUG`: "Computed value x=5" (Dev only)
    *   `INFO`: "Robot Enabled" (Normal ops)
    *   `WARN`: "Battery at 20%" (Action required soon)
    *   `ERROR`: "Camera disconnected" (Functionality lost)
    *   `FATAL`: "Motor Overheat" (Safety hazard, shutting down)

## 6. Summary

Debugging is not an afterthought; it is a discipline.
1.  **Doctor** checks the environment.
2.  **Graph** checks the architecture.
3.  **Plot** checks the physics.
4.  **Logging** checks the logic.

Now that we can record data (Bag) and inspect the system (Debugging), the final piece of the ecosystem puzzle is protection. How do we stop hackers from taking over our robot?