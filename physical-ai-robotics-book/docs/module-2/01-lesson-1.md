---
id: 01-lesson-1
title: "Building the Physics Playground"
sidebar_label: "1. Physics & Environment"
description: "Setting up Gazebo Fortress for physics simulation and spawning a robot."
keywords:
  - gazebo
  - physics
  - simulation
  - launch files
  - ros2
---

# Lesson 1: Building the Physics Playground

## 1. Introduction

Before we can teach a robot to see or think, it must exist. In the physical world, existence is given; in the digital world, it must be simulated.

The "Hello World" of humanoid robotics isn't printing text to a console—it's spawning a robot into a virtual world and watching it fall under the influence of gravity. If it falls, physics is working. If it floats, we have a problem.

In this lesson, we will build the "Physics Playground": a reliable, deterministic simulation environment using **Gazebo Fortress**. We will not worry about graphics or rendering quality yet. Our sole focus is the mathematical accuracy of forces, contacts, and joints.

## 2. Conceptual Understanding: The Physics Engine

At its heart, a simulator like Gazebo is a **Physics Engine**. It is a complex piece of software that solves differential equations to approximate the laws of physics.

### How it Works
Every simulation step (typically 1 millisecond), the engine performs a "step" of the simulation loop:

1.  **Force Accumulation**: It sums up all forces acting on every object (gravity, motor torques, wind).
2.  **Constraint Solving**: It checks for collisions (e.g., foot hitting ground) and joint limits (e.g., knee cannot bend backward). It calculates the "constraint forces" needed to prevent objects from passing through each other.
3.  **Integration**: It uses numerical integration (like Runge-Kutta) to calculate the new velocity and position of every object based on the forces and the previous state.
4.  **State Update**: It updates the position of every link in the world.

### Gazebo vs. Game Engines
Why don't we just use Unity or Unreal Engine for everything?
*   **Game Engines (Unity/Unreal)**: prioritize **frame rate**. They cheat physics to ensure the game runs at 60 FPS. They might approximate a collision to look "good enough" even if it's mathematically wrong.
*   **Robotics Simulators (Gazebo)**: prioritize **accuracy**. If the math takes 20ms to solve a complex contact, Gazebo will wait. It allows for "hard contacts" (no interpenetration), which is critical for walking robots.

## 3. System Perspective: The Architecture

To build a Digital Twin, we need three distinct systems working together: **ROS 2**, **Gazebo**, and the **Bridge**.

### The Architecture Diagram

```text
+----------------------+        +--------------------------+        +----------------------+
|       ROS 2          |        |      ROS_GZ_BRIDGE       |        |    Gazebo Fortress   |
|  (The "Brain")       |        |      (The "Translator")  |        |    (The "World")     |
|                      |        |                          |        |                      |
|  [Control Node]      |  TCP   |  [Subscriber] -> [Pub]   |  UDP   |  [Physics Engine]    |
|   /cmd_vel (Twist) --+------->+-- /cmd_vel -> /cmd_vel --+------->+-- Acts on Robot      |
|                      |        |                          |        |                      |
|  [Nav Stack]         |        |  [Publisher] <- [Sub]    |        |  [Sensors]           |
|   /scan (LaserScan) <+------- +-- /scan <---- /scan     <+------- +-- Ray Casting        |
+----------------------+        +--------------------------+        +----------------------+
```

### The Components

1.  **ROS 2 (The Brain)**: Where our control logic, navigation, and AI live. It speaks the language of DDS (Data Distribution Service) topics.
2.  **Gazebo Fortress (The World)**: The simulation server. It runs the physics loop and sensor generation. It speaks the language of Gazebo Transport (Ignition Transport).
3.  **ros_gz_bridge (The Translator)**: A critical node that sits in the middle. It takes ROS 2 messages (like `/cmd_vel`) and converts them into Gazebo messages, and vice versa. Without this, the brain cannot talk to the body.

## 4. The World Environment (SDF)

In ROS 2, we describe robots using **URDF** (Unified Robot Description Format). However, Gazebo uses **SDF** (Simulation Description Format) to describe the *environment*.

SDF is more powerful than URDF for simulation because it can describe lighting, physics parameters, and non-robot objects like buildings or terrain.

For our playground, we start with a standard "Empty World". It isn't truly empty; it contains:
1.  **Physics Definition**: Setting gravity to $-9.8 m/s^2$ in the Z-axis.
2.  **Ground Plane**: An infinite collision plane so objects don't fall forever.
3.  **Sun**: A directional light source so sensors can see.

## 5. The Launch System

We will use a Python launch file to orchestrate this system. Launch files are essential because starting a simulation involves multiple dependent processes that must be started in a specific order with specific configuration arguments.

### Creating the Launch File

Create the file `code/module-2/launch/simulation.launch.py`. This script performs three key actions:
1.  **Finds the Package**: Locates the installed `ros_gz_sim` package.
2.  **Launches Gazebo**: Starts the simulator server with an empty world.
3.  **Starts the Bridge**: configured to pass specific topics.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the ros_gz_sim package
    # This utility finds where the Gazebo ROS packages are installed on your system
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 2. Define the Gazebo launch command
    # We include the standard launch file provided by Gazebo but pass arguments
    # '-r' runs the simulation immediately (no pause)
    # 'empty.sdf' loads the default empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Define the Bridge
    # This Node forwards messages between ROS 2 and Gazebo.
    # The arguments format is 'ROS_TOPIC@ROS_TYPE[GAZEBO_TYPE'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge the /scan topic (LiDAR) from Gazebo to ROS 2
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Bridge the /cmd_vel topic (Movement) from ROS 2 to Gazebo
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen'
    )

    # 4. Return the LaunchDescription
    return LaunchDescription([
        gazebo,
        bridge
    ])
```

## 6. Real-World Example: The "Floating Robot" Problem

In 2015, during the DARPA Robotics Challenge, several teams had simulators that were "too stable." Their virtual robots could walk perfectly on flat ground.

When they deployed to the real world, the robots fell over immediately. Why?

Their simulators used simplified physics that ignored **contact compliance**. In the real world, when a metal foot hits concrete, there is a micro-bounce. The ground isn't perfectly rigid; there is a tiny amount of give.

Gazebo Fortress uses the **DART** (Dynamic Animation and Robotics Toolkit) or **ODE** physics engines, which simulate these "hard contacts" accurately. By building our playground in Gazebo, we force our control algorithms to deal with the harsh reality of discontinuous forces, making our code much more robust when we eventually move to real hardware.

## 7. Engineering Insights: The Real-Time Factor (RTF)

When you run the simulation, look for a number labeled **RTF** (Real-Time Factor) in the bottom right corner of the Gazebo window.

*   **RTF = 1.0**: The simulation is running at real-world speed. 1 simulated second = 1 real second.
*   **RTF < 1.0**: The simulation is running slower than real-time. Your computer cannot solve the physics equations fast enough.
*   **RTF > 1.0**: The simulation is "fast-forwarding."

**Critical Insight**: ROS 2 relies on time. If your RTF drops to 0.1 (10x slower), your navigation stack might timeout because it thinks the robot is stuck, even though it's just the simulation lagging. To fix this, we use "Sim Time" (`use_sim_time=True`), which forces all ROS nodes to synchronize their clocks to the simulator's clock, not the wall clock.

## 8. Summary

We have laid the foundation for our Digital Twin.
1.  We understood that **Gazebo** is our mathematical ground truth for physics.
2.  We designed an **Architecture** where ROS 2 sends commands, Gazebo simulates physics, and the Bridge translates between them.
3.  We implemented a **Launch File** to spin up this entire infrastructure with one command.

At this stage, we have an empty world. In the next lesson, we will give our robot eyes by adding **Perception Sensors** (LiDAR and Cameras) to our simulation.
