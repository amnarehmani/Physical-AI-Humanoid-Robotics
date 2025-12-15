---
id: m1-ch2-summary
title: "Chapter 2 Summary"
sidebar_label: "Summary & Quiz"
description: "Review of Services, Actions, and Parameters."
keywords:
  - ros2
  - summary
  - quiz
  - advanced concepts
---

# Chapter 2 Summary: The Cognitive Toolkit

## 1. Introduction

In Chapter 1, we built a robot that could feel and move (Topics). In Chapter 2, we gave it the ability to think, plan, and adapt.

We moved from the simple broadcast world of Topics into the sophisticated transactional world of advanced ROS 2 patterns.
*   **Services** allowed us to ask specific questions.
*   **Actions** allowed us to commit to long-term goals.
*   **Parameters** allowed us to adapt to different environments without rewriting code.

Together with Topics, these form the "Golden Triangle" of ROS 2 architecture.

## 2. System Perspective: The Complete Architecture

Let's visualize how these patterns work together in a complete robot application.

```mermaid-text
[Human Operator / High-Level Planner]
       |
       | (1) Action Goal: "Navigate to Kitchen"
       v
[Navigation Node (Action Server)]
       |
       | (2) Service Request: "Check Battery"
       +-------------------------------------> [Battery Node (Service Server)]
       |                                            |
       | <------------------------------------------+ (3) Response: "80%"
       |
       | (4) Read Parameter: "Max Speed" = 0.5 m/s
       |
       | (5) Publish Cmd: "Velocity" (Topic)
       v
[Motor Controller (Subscriber)]
```

1.  **Action**: Initiates the high-level behavior.
2.  **Service**: Performs a synchronous check before starting.
3.  **Parameter**: Configures the behavior (speed limit).
4.  **Topic**: Executes the continuous control loop.

## 3. Engineering Insights: Choosing the Right Tool

| Feature | Topic | Service | Action |
| :--- | :--- | :--- | :--- |
| **Duration** | Infinite (Stream) | Short (ms) | Long (Minutes) |
| **Feedback** | Continuous | None | Continuous |
| **Cancelable?** | No | No | Yes |
| **Analogy** | Radio | Phone Call | Taxi Ride |
| **Best For** | Sensors, Motors | State queries, Toggles | Navigation, Manipulation |

**Rule of Thumb**:
*   If you want to know **what is happening right now**, use a **Topic**.
*   If you want to **make something happen quickly**, use a **Service**.
*   If you want to **make something happen eventually**, use an **Action**.

## 4. Common Pitfalls

*   **Blocking the Thread**: Using synchronous Service calls inside a callback. This freezes the node. *Fix: Use Async calls.*
*   **Feedback Spam**: Publishing Action feedback too fast (e.g., 1000Hz). This clogs the network. *Fix: Rate limit feedback to 1-10Hz.*
*   **Hard-Coding**: Putting numbers (`speed = 0.5`) directly in Python files. *Fix: Use Parameters and Launch files.*

## 5. Mini Quiz

Test your knowledge of Chapter 2.

1.  **Scenario**: You want to tell a robotic hand to "Open Gripper" (takes 0.5s).
    *   *Best Pattern*: **Service**. It's a quick, discrete state change. You want to know when it's done.

2.  **Scenario**: You want to tell a drone to "Patrol Sector A" (takes 10 mins).
    *   *Best Pattern*: **Action**. You need to track progress and maybe cancel if battery gets low.

3.  **True or False**: Changing a Parameter value always automatically updates the running node.
    *   *Answer*: **False**. The node must explicitly implement a parameter callback to handle runtime updates. Otherwise, it only updates on restart.

4.  **Why shouldn't you use a Service for Navigation?**
    *   *Answer*: Because the client would freeze (block) for the entire duration of the movement, making the robot unresponsive to other events.

## 6. What's Next?

We have the tools. We have the code. But we are still just printing text to a terminal.

In **Chapter 3**, we will break out of the standard types (`int`, `string`, `float`). We will learn how to define **Custom Interfaces** to send data that matters to *us*—like `DetectedFace` or `MissionStatus`.