---
id: 01-lesson-1
title: "The Computing Graph"
sidebar_label: "1. Architecture"
description: "Understanding Nodes, Topics, and Services in ROS 2."
keywords:
  - ros2
  - nodes
  - topics
  - architecture
  - services
  - graph
  - dds
  - middleware
---

# Lesson 1: The Computing Graph

## 1. Introduction

In the previous lesson, we established that ROS 2 acts as the nervous system for our robot. Now, we will dissect that nervous system to understand its fundamental cells and synapses.

This lesson exists because software complexity is the silent killer of robotics projects. A humanoid robot is not a single coherent entity; it is a distributed system of independent agents working in parallel. A leg controller running at 1000Hz has very different requirements than a vision system running at 30Hz. If you try to force them into a single `while(true)` loop, your robot will stutter, lag, and eventually fall.

Understanding the **Computation Graph**—the network of Nodes (processing units) and Topics (data streams)—is the difference between a fragile prototype and a robust product. This architecture allows us to build systems where a vision failure doesn't paralyze the legs, and where new sensors can be added without rewriting the brain.

## 2. Conceptual Understanding: The Anatomy of Decentralization

### 2.1 The Problem with Monoliths
Imagine writing a Python script to control a humanoid. You might start with a simple loop:
1.  Read Camera.
2.  Find Face.
3.  Calculate Joint Angles.
4.  Move Motors.

This works for a toy. But what happens when "Find Face" takes 200 milliseconds? The loop blocks. The "Move Motors" step is delayed. The robot loses balance and falls. You could try multi-threading, but then you face race conditions, deadlocks, and shared memory corruption.

### 2.2 The Solution: The Graph
ROS 2 solves this by breaking the application into small, isolated processes called **Nodes**.
*   **Isolation**: Each node has its own memory space. If the "Vision Node" crashes (segfaults), the "Balance Node" keeps running.
*   **Asynchrony**: Nodes run at their own speeds. The Balance Node can loop at 1000Hz while the Vision Node loops at 30Hz.
*   **Location Agnostic**: Nodes don't know if they are on the same CPU, a different core, or a different computer entirely.

### 2.3 Nodes (The Organs)
**Intuition**: Think of a Node as a specialist employee in a large company.
*   The **Camera Driver Node** is the photographer—it only takes pictures.
*   The **Face Detector Node** is the analyst—it only looks for patterns.
*   The **Motor Controller Node** is the mechanic—it only moves gears.

They don't know how to do each other's jobs, but they excel at their own. They don't even know each other's names; they just broadcast their work and listen for instructions.

### 2.4 Topics (The Nerves)
**Intuition**: Topics are the nerves carrying signals. They follow a **Publish-Subscribe** pattern.
*   **Publisher**: Broadcasts information (e.g., "The temperature is 25°C"). It shouts into the void, unaware of who is listening.
*   **Subscriber**: Tunes into a specific channel (e.g., "Tell me the temperature").

This decoupling is critical. You can add a "Data Logger" node that subscribes to the camera stream without changing a single line of code in the Camera Driver.

## 3. System Perspective: Data Flow and Middleware

How does this actually work under the hood?

### 3.1 The Middleware (DDS)
ROS 2 is built on top of **DDS (Data Distribution Service)**, an industrial-grade networking standard used in battleships and financial trading systems.
*   When a Node publishes a message, it doesn't just hand it to another function. It serializes it into a binary packet.
*   DDS handles the transport (UDP/TCP), discovery (finding other nodes), and Quality of Service (QoS).

### 3.2 The Message Lifecycle
1.  **Generation**: The Camera Node captures a frame and populates an `Image` message object.
2.  **Serialization**: The middleware converts this object into a byte stream.
3.  **Transport**: The bytes traverse the network stack (localhost or Ethernet).
4.  **Deserialization**: The receiving middleware reconstructs the object.
5.  **Callback**: The Subscriber Node's **Executor** wakes up and triggers the defined callback function (e.g., `process_image(msg)`).

### 3.3 Quality of Service (QoS)
Not all nerves are equal.
*   **Reliable (TCP-like)**: Guaranteed delivery. Used for mission-critical commands like "Emergency Stop". If a packet is dropped, it retries.
*   **Best Effort (UDP-like)**: Fire and forget. Used for high-frequency sensor data like "Video Stream". If you miss a frame from 0.01 seconds ago, you don't want it re-sent; you want the *new* frame.

## 4. System Architecture Diagram

Below is the architecture for a "Head Tracking" system. Notice the feedback loop and the separation of concerns.

```mermaid-text
+-----------------------+           +-----------------------+
|  Node: Camera_Driver  |           |  Node: Face_Detector  |
|                       |           |                       |
|   [Task: Read HW]     |           |   [Task: Find Face]   |
|           |           |           |           ^           |
|  (Pub) /image_raw     |---------->|  (Sub) /image_raw     |
|           |           |           |           |           |
+-----------------------+           |  (Pub) /face_coords   |
                                    |           |           |
                                    +-----------+-----------+
                                                |
                                                v
                                    +-----------------------+
                                    |  Node: Head_Controller|
                                    |                       |
                                    |   [Task: Move Motors] |
                                    |           ^           |
      (Physical Motion)             |  (Sub) /face_coords   |
              ^                     |           |           |
              |                     |   (Pub) /motor_cmd    |
              +---------------------|           |           |
                                    +-----------------------+
```

1.  **Camera_Driver**: Pure I/O. Reads hardware, publishes heavy data.
2.  **Face_Detector**: Pure Compute. Consumes heavy data, produces lightweight coordinates.
3.  **Head_Controller**: Pure Logic. Consumes coordinates, produces motor commands.

## 5. Practical Example: The "Fall Detection" Node

Consider the safety system of a walking humanoid.

*   **Scenario**: The robot trips over a cable.
*   **IMU Node**: Reads the gyroscope/accelerometer 500 times a second. It publishes `/imu/data` using **Best Effort** QoS (low latency).
*   **Fall Detector Node**: Subscribes to `/imu/data`. It checks the pitch angle.
    *   *Logic*: `if pitch > 45_degrees: status = FALLING`
    *   *Decision*: If falling, it triggers a Service Call to the **Safety Node**: `enable_protective_crouch()`.
*   **Safety Node**: Immediately overrides all walking commands, stiffens the joints, and protects the head.

**Why this structure?**
Speed and Safety. The IMU node is simple and fast. The Fall Detector allows us to isolate the specific logic of "what constitutes a fall." The Safety Node has the ultimate authority to override the "Walking Node," ensuring that no matter what the high-level planner thinks, the robot protects itself.

## 6. Engineering Insights

When designing your graph, you face critical trade-offs:

*   **Monolith vs. Micro-nodes**:
    *   *Monolith*: Zero latency, shared memory, but fragile and hard to debug.
    *   *Micro-nodes*: Robust, modular, easy to test, but introduces network overhead (serialization cost).
    *   *Insight*: Use **Nodelets** (Intra-process communication) for high-bandwidth data (like video) to get the best of both worlds (zero-copy transfer with modular code).

*   **Bandwidth**:
    *   *Constraint*: Sending 4K uncompressed video over a Topic can saturate your CPU and network.
    *   *Insight*: Process data as close to the source as possible. Don't send raw images to a central brain; send "Bounding Boxes" or "Feature Vectors" which are kilobits, not megabytes.

## 7. Summary

*   **Nodes** are the independent workers of your robot.
*   **Topics** are the continuous broadcast channels.
*   **Services** are for request/response interactions.
*   **The Graph** is the map of how data flows through your system.

We have defined the anatomy. We understand *why* we separate the brain from the eyes and the muscles. In the next lesson, we will write the code to make these nodes talk.
