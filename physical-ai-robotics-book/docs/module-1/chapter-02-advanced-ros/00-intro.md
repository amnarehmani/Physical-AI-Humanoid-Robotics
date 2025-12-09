---
id: m1-ch2-intro
title: "Chapter 2: Advanced ROS 2 Concepts"
sidebar_label: "Introduction"
description: "Moving beyond topics to Services, Actions, and Parameters."
keywords:
  - ros2
  - services
  - actions
  - parameters
  - middleware
---

# Chapter 2: Advanced ROS 2 Concepts

## Introduction

In Chapter 1, we established the fundamental concept of **Topics**—a continuous stream of data flow, analogous to a television broadcast or a nerve impulse carrying sensory data. While Topics are excellent for sensor streams (camera feeds, joint states) or continuous control commands (velocity targets), they lack the mechanisms required for transactional or goal-oriented behaviors.

Imagine a robot arm. Sending a continuous stream of "open gripper" commands at 100Hz is inefficient. Instead, you want to send a single command: "Open Gripper," and receive a confirmation: "Gripper Opened." This is a **transaction**.

Furthermore, imagine telling a mobile robot to "Go to the Kitchen." This is not a transaction that happens instantly; it takes time. The robot might need to report progress ("I am 50% there") or allow you to cancel the command if the kitchen is suddenly on fire. This is a **Goal**.

In this chapter, we will expand our ROS 2 vocabulary to include **Services**, **Actions**, and **Parameters**, giving us the complete toolkit to build complex robotic behaviors.

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Differentiate** between Topics, Services, and Actions based on communication patterns (Streaming vs. Transactional vs. Goal-oriented).
2.  **Implement** a ROS 2 Service Server and Client in Python to handle request-response logic.
3.  **Construct** an Action Server to execute long-running tasks with feedback and cancellation capabilities.
4.  **Manage** node configuration dynamically using ROS 2 Parameters and Launch files.

## Tools & Prerequisites

*   **ROS 2 Distribution**: Humble Hawksbill or Iron Irwini (or newer).
*   **Language**: Python 3.10+ (`rclpy`).
*   **System**: The workspace created in Chapter 1.
*   **Concept**: Understanding of Python `async`/`await` patterns is helpful but will be explained.

## Technical Context: The Communication Trinity

| Pattern | Analogy | Description | Use Case |
| :--- | :--- | :--- | :--- |
| **Topic** | Radio Broadcast | One-way, continuous streaming. Fire and forget. | Sensor data, Teleoperation. |
| **Service** | Web Request | Two-way, synchronous transaction. Request -> Response. | "Spawn object", "Reset simulation". |
| **Action** | Uber Ride | Asynchronous, goal-oriented. Request -> Feedback... -> Result. | Navigation, Manipulation. |

![diagram](pathname://placeholder-ros2-communication-patterns)
*Figure 2.1: Visual comparison of Topic, Service, and Action architectures.*

## Real-World Robotics Use Cases

### 1. The Service: "Calibrate IMU"
A robot's Inertial Measurement Unit (IMU) drifts over time. A background process monitors this. When the robot is stationary, the "Brain" sends a **Service Request** to the "IMU Node": `Calibrate(timeout=5s)`. The IMU Node pauses data streaming, recalibrates, and sends a **Service Response**: `Success`. This prevents the "Brain" from acting on bad data during calibration.

### 2. The Action: "Pick Up Object"
A humanoid robot identifies a cup. The high-level planner sends an **Action Goal** to the arm controller: `Pick(target=cup_pose)`.
*   **Feedback**: The arm controller reports "Planning path...", "Moving (50% complete)...", "Closing gripper...".
*   **Result**: "Object grasped successfully."
If the object slips, the controller can abort and report `Result: Failed`. This rich interaction is impossible with simple Topics.

## Code Preview

In this chapter, we will be writing Python nodes that look like this:

```python
# A Service Server Example
def callback(self, request, response):
    self.get_logger().info(f'Incoming request: {request.a} + {request.b}')
    response.sum = request.a + request.b
    return response
```

We will build these step-by-step. Let's begin by mastering the Service.
