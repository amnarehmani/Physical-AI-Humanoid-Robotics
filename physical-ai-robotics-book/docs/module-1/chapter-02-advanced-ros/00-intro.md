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

## 1. Introduction

In Chapter 1, we built a nervous system that could **stream** information. Like a heart beating or an eye seeing, our nodes broadcasted data continuously. This is powerful, but it is primitive.

Imagine a human who can only "stream." They can shout "I am hungry!" every second, but they cannot ask a specific question like "Do you have an apple?" and wait for a "Yes/No" answer. They cannot commit to a long-term goal like "Walk to the store" and report their progress along the way.

To build a truly intelligent humanoid, we need more than just continuous streams. We need **conversation** and **commitment**.

This chapter introduces the three advanced communication patterns that turn a reactive robot into a cognitive one:
1.  **Services**: For immediate questions and commands.
2.  **Actions**: For long-term goals with feedback.
3.  **Parameters**: For configuring the robot's personality and physics.

## 2. Conceptual Understanding: The Communication Trinity

We established that **Topics** are like a **Radio Broadcast**. Now, let's introduce the new players using human analogies.

### The Service (The Phone Call)
*   **Analogy**: You call a pizza place. You ask, "Are you open?" You wait. They say, "Yes." You hang up.
*   **Mechanism**: **Synchronous Request-Response**.
*   **Why we need it**: Sometimes you need a specific answer *right now*. A topic doesn't guarantee a response; a service does.
*   **Robotics Example**: "Reset Simulation", "Calibrate Sensor", "Spawn Object".

### The Action (The Taxi Ride)
*   **Analogy**: You get in a taxi and say, "Take me to the airport."
    *   The driver doesn't say "Done" immediately (that would be teleportation).
    *   Instead, the driver accepts the goal.
    *   During the drive, you can see the GPS (Feedback: "10 minutes remaining").
    *   If you change your mind, you can say "Stop, let me out" (Cancel).
    *   Finally, you arrive (Result: "Success").
*   **Mechanism**: **Asynchronous Goal with Feedback**.
*   **Why we need it**: Real-world tasks take time. Navigating a room takes 30 seconds. Grasping a cup takes 5 seconds. We need a way to track progress and cancel if things go wrong.

## 3. System Perspective: Architecture Comparison

Let's visualize how these patterns differ in data flow.

```mermaid-text
       TOPIC (Streaming)              SERVICE (Transaction)
    +-------+      +-------+       +-------+      +-------+
    | Node A| ---> | Node B|       | Client| ---> | Server|
    +-------+      +-------+       +-------+      +-------+
    (Continuous Data Flow)         (Request)      (Process)
                                       |              |
                                   (Wait) <---------- (Response)


                ACTION (Goal-Oriented)
    +-------+                            +-------+
    | Client| -------------------------> | Server|
    +-------+       (1. Goal)            +-------+
       |                                     |
       | <---------------------------------- |
       |       (2. Accept/Reject)            |
       |                                     |
       | <---------------------------------- |
       |       (3. Feedback... 10%... 50%)   |
       |                                     |
       | ----------------------------------> |
       |       (4. Cancel? Optional)         |
       |                                     |
       | <---------------------------------- |
       |       (5. Final Result)             |
    +-------+                            +-------+
```

## 4. Real-World Humanoid Scenarios

### Scenario A: The "Wake Up" Routine (Service)
When you turn on the robot, the motors are limp. The "Brain" needs to engage the brakes and stiffen the joints.
*   **Bad Approach (Topic)**: Publish `stiffness=100` continuously. You don't know if the motors actually received it or if they are ready.
*   **Good Approach (Service)**: Call `/engage_motors`. The Motor Controller checks voltages, runs safety diagnostics, engages the brakes, and returns `success=True`. The Brain knows for sure the robot is ready to stand.

### Scenario B: The "Fetch Coffee" Task (Action)
The Brain decides to get coffee.
*   **Bad Approach (Service)**: Call `/get_coffee`. The calling function **blocks** (freezes) for 5 minutes while the robot walks. The robot cannot see obstacles or react to people because its brain is frozen waiting for the return value.
*   **Good Approach (Action)**: Send goal `/get_coffee`.
    *   The Brain continues processing vision.
    *   The Action Server reports: "Walking to kitchen..."
    *   Suddenly, a child runs in front.
    *   The Brain sends: **Cancel Goal**.
    *   The robot stops immediately.

## 5. Learning Objectives

By the end of this chapter, you will be able to:

1.  **Architect** complex behaviors by choosing the right pattern (Topic vs Service vs Action).
2.  **Implement** a Custom Service to handle specific requests.
3.  **Construct** an Action Server that can execute time-extended tasks and report progress.
4.  **Configure** your nodes dynamically using ROS 2 Parameters, avoiding hard-coded values.

We are moving from simple reflexes to complex, deliberate behavior. Let's start by learning how to ask questions with **Services**.