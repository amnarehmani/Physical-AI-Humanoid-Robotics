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

<h2>1.1 Introduction to Gazebo Fortress</h2>

**Gazebo** is the industry standard for robot simulation. We are using **Gazebo Fortress** (part of the new Gazebo, formerly "Ignition"), which is designed specifically for ROS 2 integration.

Gazebo's job is to solve differential equations. Every millisecond, it asks:
*   "Is gravity pulling this link down?"
*   "Did this wheel hit the ground?"
*   "How much friction is on this surface?"

<h2>1.2 The World File (SDF)</h2>

Gazebo uses **SDF** (Simulation Description Format) for worlds. While ROS uses URDF for robots, Gazebo uses SDF for environments.

We will use a standard empty world to start. It contains:
1.  **Physics**: Settings for gravity (-9.8 m/s^2).
2.  **Ground Plane**: An infinite floor.
3.  **Sun**: A light source.

<h2>1.3 The Launch File</h2>

We need to tell ROS 2 to:
1.  Start Gazebo.
2.  Load our World.
3.  Convert our URDF to something Gazebo understands (spawn).
4.  Start the **ROS-Gazebo Bridge** (so topics can pass through).

This is complex, so we script it in Python.

<h3>Creating the Launch File</h3>

Create `code/module-2/launch/simulation.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the ros_gz_sim package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 2. Define the Gazebo launch command
    # We launch 'gz_sim.launch.py' and pass arguments to load an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. Define the Bridge
    # This Node forwards messages between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Bridge the /scan topic (LiDAR)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Bridge the /cmd_vel topic (Movement commands)
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

<h2>1.4 Exercise: Ignition</h2>

Run the launch file in your terminal.

```bash
ros2 launch simulation.launch.py
```

**Success Criteria**:
1.  A Gazebo window opens.
2.  You see a grid (the ground plane).
3.  Wait... where is the robot?

We haven't spawned it yet! In a full setup, we would add a `create` node to this launch file to inject the URDF. For now, verify that the simulator itself is running.