---
id: m1-ch5-intro
title: "Chapter 5: Project - TurtleBot3 Patrol"
sidebar_label: "Introduction"
description: "A capstone project integrating Topics, Actions, and Transforms."
keywords:
  - turtlebot3
  - capstone
  - project
  - navigation
  - nav2
---

# Chapter 5: Project - TurtleBot3 Patrol

## 1. Introduction

We have spent four chapters learning the vocabulary of ROS 2. We know the words (Topics, Services, Actions), the grammar (Interfaces, TF2), and the tools (Rviz, Bag). Now, it is time to write a story.

In this Capstone Project, we will build a complete **Autonomous Security Guard**.
We will simulate a TurtleBot3 that patrols a house endlessly. It will drive to the kitchen, scan for intruders (spin around), drive to the living room, scan again, and repeat.

This is not a toy example. This architecture—a state machine orchestrating navigation and perception—is identical to what runs on $50,000 industrial mobile robots in warehouses today.

## 2. Conceptual Understanding: The Integration Challenge

**Intuition**:
Building a robot is like conducting an orchestra.
*   **The Musicians**: Nav2 (moves the robot), Lidar (sees the world), Motors (spin the wheels).
*   **The Conductor**: Your Code.

Your code doesn't move the wheels directly; that's Nav2's job. Your code tells Nav2 *where* to go. Your code doesn't parse raw laser bytes; it waits for the Lidar driver to publish `/scan`.
The challenge is **Timing**. You cannot start scanning until you arrive. You cannot leave until scanning is done.

## 3. System Perspective: The Architecture

We will build a modular system consisting of three custom nodes interacting with the standard ROS 2 ecosystem.

```mermaid-text
      [System Config (YAML)] --> (Waypoints)
                                     |
                                     v
                            [Patrol Node (Main Brain)]
                                     |
        +----------------------------+-----------------------------+
        | (Action: Navigate)         | (Service: Start Scan)       | (Topic: Status)
        v                            v                             v
[Nav2 Stack (Standard)]      [Scanner Node (Custom)]       [Dashboard Node (UI)]
        |                            |
        v (Cmd_Vel)                  v (Cmd_Vel)
   [Robot Hardware / Gazebo Simulation]
```

1.  **Patrol Node**: The State Machine. It decides *what* to do next.
2.  **Scanner Node**: The Perception unit. It spins the robot and looks at Lidar data.
3.  **Dashboard Node**: The Remote Monitor. It displays "Battery: 80%, Status: Patrolling Kitchen".

## 4. Real-World Humanoid Scenarios

### Scenario A: Warehouse Security
Security robots (like Knightscope) patrol parking lots. They use GPS/Lidar for navigation (Nav2), thermal cameras for anomaly detection (Scanner Node), and 4G to report to HQ (Dashboard Node).

### Scenario B: Hospital Logistics
A Tug robot moves laundry. It navigates hallways (Nav2), docks with carts (Scanner Node/Fine alignment), and updates the nurse station (Dashboard Node).

## 5. Learning Objectives

By the end of this chapter, you will be able to:

1.  **Orchestrate** complex behaviors using a Finite State Machine (FSM).
2.  **Interface** with the standard Navigation 2 (Nav2) stack using Action Clients.
3.  **Arbitrate** control: Ensure the "Scanner" and "Navigator" don't fight over the wheels.
4.  **Launch** a multi-node, multi-package system with a single command.

We start by building the hardest part: The Interface to the Navigation Stack.