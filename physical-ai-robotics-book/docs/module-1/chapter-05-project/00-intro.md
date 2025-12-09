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
---

# Chapter 5: Project - TurtleBot3 Patrol

## Introduction

Welcome to the Module 1 Capstone. We have learned the theory; now we build a product.

We will create a **Patrol Application** for a TurtleBot3. The robot will:
1.  Navigate between a set of waypoints (Kitchen, Living Room).
2.  Pause at each location to "Scan" (spin 360 degrees).
3.  Report its status back to a base station.

This project combines **Topics** (Odom, Scan), **Actions** (NavigateToPose), **Services** (Reset), and **Parameters** (Waypoints).

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Architect** a multi-node ROS 2 application.
2.  **Integrate** the Nav2 (Navigation 2) stack via Action Clients.
3.  **Manage** robot state using a Finite State Machine (FSM).
4.  **Launch** a complex system with a single command.

## Tools & Prerequisites

*   **TurtleBot3 Simulation**: `sudo apt install ros-humble-turtlebot3*`
*   **Nav2**: `sudo apt install ros-humble-navigation2`
*   **Behavior Tree** concept (optional but helpful).

## The Architecture

We will build three nodes:
1.  **PatrolNode**: The brain. Reads parameters (waypoints), sends Action Goals to Nav2.
2.  **ScannerNode**: The eyes. When requested (Service), spins the robot and checks Lidar for intruders.
3.  **DashboardNode**: The UI. Subscribes to status topics and prints a clean report.

![diagram](pathname://placeholder-turtlebot-patrol-arch)
*Figure 5.1: System Architecture of the Patrol Bot.*

## Real-World Robotics Use Cases

### 1. Security Robots
Companies like Knightscope build robots that do exactly this: patrol a parking lot, scan license plates, and report anomalies.

### 2. Hospital Delivery
A robot moves from Room 101 to Room 102. At each room, it announces "Delivery Arrived" (Action) and waits for confirmation (Service) before moving on.

Let's begin by setting up the simulation environment.
