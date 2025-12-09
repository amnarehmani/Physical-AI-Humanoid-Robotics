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

## Introduction

So far, we have used standard data types: `Int64`, `String`, `Twist`. But real robots produce unique data. A "Face Detection" node might need to output a message containing `string person_name`, `float32 confidence`, and `int32[4] bounding_box`. Standard types cannot handle this efficiently.

Moreover, robots exist in 3D space. If a camera on the robot's head sees a cup 1 meter away, where is that cup relative to the robot's hand? To solve this, we need **TF2 (The Transform Library)**.

This chapter bridges the gap between simple logic and physical reality.

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Define** custom `.msg` and `.srv` files to structure your data.
2.  **Configure** `CMakeLists.txt` and `package.xml` to generate code for these interfaces.
3.  **Visualize** the robot's coordinate frames using the TF2 toolset.
4.  **Broadcast** and **Listen** to coordinate transforms to convert points from one frame to another.

## Tools & Prerequisites

*   **Rviz2**: The standard ROS 2 visualization tool.
*   **C++ Build Tools**: Even for Python users, interfaces are generated using CMake logic.

## The Concept of TF2

Robotics is largely about coordinate algebra.
*   **Map Frame**: Fixed in the world (0,0,0).
*   **Odom Frame**: The robot's start point.
*   **Base Link**: The center of the robot chassis.
*   **Camera Link**: The lens of the camera.

If the camera is mounted 0.5m above the chassis, there is a **Transform** from `base_link` to `camera_link` (z = +0.5). TF2 manages these relationships continuously, allowing you to ask: "What is the position of the cup (seen by camera) in the coordinates of the gripper?"

Let's start by defining our data structures.
