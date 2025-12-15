---
id: m2-ch4-spawning
title: "Lesson 2: Spawning Robots"
sidebar_label: "Lesson 2: Spawning"
description: "Using ros_gz_sim to inject URDFs into the world."
keywords:
  - spawn
  - urdf
  - create
  - ros_gz_sim
---

# Lesson 2: Spawning Robots

## 1. Introduction

In classic Gazebo, we often loaded the robot as part of the world file. In modern Gazebo (Fortress/Harmonic), we prefer to **Spawn** robots dynamically.

Why?
1.  **Modularity**: The world file describes the environment (walls, lights). The robot is a separate entity.
2.  **Multi-Robot Systems**: You can spawn 5 robots in a loop without writing a world file with 5 robot blocks.
3.  **Reset**: You can delete and re-spawn a robot without restarting the simulator.

## 2. Conceptual Understanding: The Factory Service

Gazebo exposes a "Factory" service.
1.  **ROS Node (`create`)**: Reads your URDF.
2.  **Conversion**: Converts URDF to SDF (Gazebo's native format).
3.  **Service Call**: Sends the SDF XML to the Gazebo Factory service.
4.  **Instantiation**: Gazebo allocates memory and adds the model to the physics loop.

## 3. System Perspective: The Launch Pipeline

Spawning is a sequence of dependencies.

```text
      [ 1. Launch Gazebo ]  <-- (Empty World)
               |
      [ 2. Robot State Publisher ]  <-- (Reads URDF, publishes /robot_description)
               |
      [ 3. Spawn Node ]  <-- (Subscribes to /robot_description, calls Factory)
```

If Step 2 fails, Step 3 waits forever.

## 4. Implementation: The `create` Node

We use the `create` executable from `ros_gz_sim`.

### CLI Usage
```bash
ros2 run ros_gz_sim create -topic robot_description -name my_bot -x 2.0 -z 0.5
```

### Launch File Usage
```python
# spawn.launch.py
spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-topic', 'robot_description', # Read URDF from this topic
        '-name', 'my_robot',           # Name in Gazebo
        '-x', '0.0',
        '-y', '0.0',
        '-z', '0.3'                    # Drop height
    ],
    output='screen'
)
```

## 5. Engineering Insights: Mesh Paths

The #1 error in spawning is **Missing Meshes**.
Gazebo runs in a different process environment than ROS.
*   **ROS**: Sees `package://my_robot/meshes/wheel.stl`.
*   **Gazebo**: Might not know where `my_robot` is.

**Fix**: You must ensure the `GAZEBO_MODEL_PATH` (or `IGN_GAZEBO_RESOURCE_PATH`) environment variable includes your workspace install folder.
`export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/path/to/ws/install/share`

## 6. Real-World Example: Random Initialization

In Reinforcement Learning (RL), we never spawn the robot in the same place twice.
We write a Python script that generates random `x, y, yaw` coordinates and calls the spawn command.
```python
x = random.uniform(-5, 5)
y = random.uniform(-5, 5)
spawn_cmd = f"ros2 run ros_gz_sim create ... -x {x} -y {y}"
```
This forces the robot to learn general navigation, not just "memorize the path from 0,0".

## 7. Summary

Spawning is how we inject our agent into the matrix.
By separating the **World** (Environment) from the **Robot** (Agent), we gain the flexibility to test different robots in the same world, or the same robot in different worlds.

In the next lesson, we will tackle the final piece of the bridge puzzle: **Time**.