---
id: m1-ch5-launch
title: "Lesson 3: Launch and Test"
sidebar_label: "Lesson 3: Launch & Test"
description: "Creating the final launch file and running the demo."
keywords:
  - launch
  - testing
  - simulation
  - turtlebot3
  - orchestration
---

# Lesson 3: Launch and Test

## 1. Introduction

We have the Brain (Patrol Node), the Eyes (Scanner Node), and the Legs (Nav2). But currently, they are scattered scripts. To run this system, you would need to open 6 different terminal tabs and type commands in perfect synchronization. That is not scalable.

In this lesson, we will write the **Master Launch File**. This single script will act as the "ON" switch for our entire robot application, orchestrating the simulation, navigation stack, and our custom logic.

## 2. Conceptual Understanding: The Orchestrator

**Intuition**:
Think of a Launch File as a **Flight Checklist**.
*   Engine Start (Gazebo).
*   Navigation Systems Online (Nav2).
*   Flight Computer Active (Patrol Node).
*   Cockpit Displays On (Rviz).

It ensures that subsystems start in the correct order and with the correct configuration.

## 3. System Perspective: The Launch Tree

Our `patrol_app.launch.py` will not start nodes directly; it will include other launch files.

```mermaid-text
[patrol_app.launch.py]
       |
       | (1) Include
       v
[nav2_bringup/tb3_simulation_launch.py]
       |
       +---> [Gazebo Server] (Physics)
       +---> [Robot State Publisher] (URDF)
       +---> [Nav2 Stack] (Planner, Controller, Map Server)
       +---> [Rviz2] (Visualization)
       |
       | (2) Start Node
       v
[patrol_node] (Our Logic)
```

## 4. Practical Example: The Master Launch File

Create `code/module-1/patrol_app.launch.py`.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the standard Nav2 launch file
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_pkg_dir, 'launch', 'tb3_simulation_launch.py')

    # 2. Define Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    headless = LaunchConfiguration('headless', default='False')

    return LaunchDescription([
        # 3. Include the Standard Nav2 Simulation
        # This handles Gazebo, Rviz, and Nav2 nodes automatically
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'headless': headless,
                'map': '/path/to/your/map.yaml' # Optional: Provide a map
            }.items(),
        ),
        
        # 4. Start Our Patrol Brain
        Node(
            package='my_patrol_pkg',
            executable='patrol_node',
            name='patrol_brain',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'waypoints': [1.0, 0.0, 0.0,  2.0, 2.0, 1.57] 
            }]
        )
    ])
```

## 5. Engineering Insights: Debugging the Launch

Launching complex systems often fails silently. Here is your checklist:

*   **The "Initial Pose" Problem**:
    *   *Symptom*: Nav2 starts, but the robot in Rviz is lost (costmap is empty).
    *   *Fix*: Nav2 needs to know where it is starting. Click "2D Pose Estimate" in Rviz and drag an arrow where the robot is in Gazebo.
*   **The "Clock" Problem**:
    *   *Symptom*: TF errors saying "Lookup would require extrapolation into the future".
    *   *Fix*: Ensure `use_sim_time` is `True` for **EVERY** node. If one node uses Wall Clock and another uses Sim Clock, they cannot talk.
*   **The "Stuck" Problem**:
    *   *Symptom*: Robot plans a path but doesn't move.
    *   *Fix*: Check if the `Controller` server is active. Is the robot in a collision state? Check the terminal for `[nav2_controller]: Failed to make progress`.

## 6. Summary

You have done it. You have gone from writing a simple "Hello World" node to orchestrating a full autonomous navigation application.

1.  **Launch Files** automate the startup.
2.  **Parameters** configure the behavior.
3.  **Nav2** handles the heavy lifting of movement.
4.  **FSM** handles the logic.

This concludes **Module 1**. You have built the nervous system.
In **Module 2**, we will leave the safe world of 2D TurtleBots and enter the complex world of **Humanoids** in high-fidelity physics.