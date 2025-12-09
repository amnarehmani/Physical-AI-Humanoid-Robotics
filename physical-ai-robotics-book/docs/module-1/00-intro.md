---
id: 00-intro
title: "Introduction to ROS 2"
sidebar_label: "Introduction"
description: "Overview of Module 1, covering ROS 2 as the robot's nervous system."
keywords:
  - ros2
  - introduction
  - middleware
  - python
---

# Module 1: The Robotic Nervous System (ROS 2)

## Introduction

Welcome to the first technical module of our journey. Before we can build intelligent agents, we need a way for them to communicate with the robot's body. 

In biological systems, the nervous system transmits signals from eyes to brain and brain to muscles. In robotics, this role is filled by **ROS 2 (Robot Operating System)**. Despite the name, ROS 2 is not an operating system like Windows or Linux; it is "Middleware"—a plumbing layer that manages communication between different programs.

### Learning Objectives

By the end of this module, you will be able to:
1.  **Conceptualize** the robot as a graph of connected nodes.
2.  **Implement** Publishers and Subscribers using Python (`rclpy`).
3.  **Define** a robot's physical structure using URDF.
4.  **Execute** a complete communication loop between a "Brain" script and a "Muscle" script.

### Prerequisites

*   **OS**: Ubuntu 22.04 (recommended) or Windows with WSL2.
*   **Python**: Intermediate (Classes, inheritance).
*   **Terminal**: Comfortable navigating directories and running commands.

### Roadmap

*   **Lesson 1**: Architecture. We break down the system into Nodes and Topics.
*   **Lesson 2**: Control. We write Python code to send signals.
*   **Lesson 3**: Body. We define the shape and physics of our robot in XML.

Let's start building the nervous system.
