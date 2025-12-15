---
id: m1-ch3-intro
title: "Chapter 3: Interfaces & Transforms"
sidebar_label: "Introduction"
description: "Defining custom data structures and managing 3D space with TF2."
keywords:
  - ros2
  - interfaces
  - msg
  - srv
  - tf2
  - transforms
---

# Chapter 3: Custom Interfaces & Transforms

## 1. Introduction

In Chapter 1 and 2, we built a robot that could think and communicate. However, its vocabulary was limited to basic words like `String` and `Integer`. Imagine trying to describe a human face using only numbers—it’s inefficient and confusing.

Furthermore, our robot existed in a vacuum. It had no concept of "where" things were. If its camera saw a cup, it didn't know if that cup was within reach of its arm.

This chapter introduces two critical capabilities:
1.  **Custom Interfaces**: The ability to define complex, domain-specific data structures (e.g., `FaceDetection`, `BatteryStatus`).
2.  **TF2 (Transform Library)**: The ability to understand 3D space and the relationships between different body parts.

## 2. Conceptual Understanding

### Data Typing (The Dictionary)
Just as a programming language has structs or classes, ROS 2 allows us to define **Custom Messages**.
*   **Problem**: Sending `[1.0, 2.5, 3.0]` over a topic is ambiguous. Is it a position? A color? A velocity?
*   **Solution**: Define a `Point3D` message with fields `x`, `y`, `z`. Now the data carries meaning and context.

### Spatial Awareness (The Skeleton)
Robotics is fundamentally a geometry problem.
*   **Analogy**: You are driving a car.
    *   Your GPS says the destination is "North".
    *   Your eyes see the road "Ahead".
    *   Your hands turn the wheel "Left".
*   **The Transform**: To navigate, your brain continuously transforms "North on GPS" into "Left on Wheel".
*   **TF2**: This is the ROS 2 subsystem that tracks the position of every part of the robot (and the world) relative to every other part over time.

## 3. System Perspective: The TF Tree

TF2 organizes the world into a tree of coordinate frames.

```mermaid-text
        [map] (World Origin)
          |
          v
       [odom] (Robot Start Point)
          |
          v
     [base_link] (Robot Center)
          |
          +------------------------+
          |                        |
          v                        v
    [camera_link]             [arm_link]
          |                        |
          v                        v
    [optical_frame]           [gripper_tip]
```

*   If the camera sees an object at `(x=1, y=0)` in `camera_link`...
*   And the camera is mounted `0.5m` above `base_link`...
*   TF2 calculates that the object is at `(x=1, y=0, z=0.5)` in `base_link`.

## 4. Real-World Humanoid Scenarios

### Scenario A: "Hand-Eye Coordination"
A humanoid sees a cup. The camera reports the cup is at `(100, 200)` pixels.
1.  **Custom Interface**: The Vision Node publishes a `DetectedObject` message containing class="cup" and pixel coordinates.
2.  **Transform**: The "Grasp Planner" asks TF2: "Where is that pixel in 3D space relative to the Right Hand?"
3.  **Action**: TF2 performs the matrix multiplication chain (Camera -> Head -> Torso -> Shoulder -> Arm -> Hand) instantly.

### Scenario B: "Fleet Management"
You have a fleet of 10 delivery robots.
1.  **Custom Interface**: You define a `DeliveryStatus` message (`order_id`, `eta`, `current_zone`).
2.  **Benefit**: Every robot speaks the same language. The central server doesn't need to parse raw strings; it deserializes structured data.

## 5. Learning Objectives

By the end of this chapter, you will be able to:

1.  **Architect** data structures using `.msg` and `.srv` files.
2.  **Generate** language-specific code (Python/C++) from these definitions using CMake.
3.  **Visualise** the robot's "skeleton" (TF tree) using **Rviz2**.
4.  **Compute** spatial relationships programmatically to enable hand-eye coordination.

We start by expanding our robot's vocabulary. Let's define our first Custom Interface.