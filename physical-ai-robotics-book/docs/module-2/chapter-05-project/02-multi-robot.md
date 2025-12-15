---
id: m2-ch5-multi-robot
title: "Lesson 2: Multi-Robot Launch"
sidebar_label: "Lesson 2: Multi-Robot"
description: "Spawning robots with namespaces."
keywords:
  - namespaces
  - multi-robot
  - tf_prefix
  - launch
---

# Lesson 2: Multi-Robot Launch

## 1. Introduction

In a real warehouse, there isn't just one robot; there are hundreds.
If we launch two robots with the default configuration, they will both publish to `/scan` and `/cmd_vel`. Robot A's sensor data will control Robot B. Chaos.

To solve this, we use **ROS Namespaces** and **TF Prefixes**.

## 2. Conceptual Understanding: The Namespace Tree

Namespaces segregate the ROS graph.

**Without Namespaces:**
*   `/cmd_vel` (Ambiguous)
*   `/scan` (Ambiguous)

**With Namespaces:**
*   `/worker_1/cmd_vel`
*   `/worker_1/scan`
*   `/worker_2/cmd_vel`
*   `/worker_2/scan`

Each robot becomes a self-contained island.

## 3. System Perspective: TF Prefixing

The Transform (TF) tree is global. We cannot have two `base_link` frames.
We must prefix the frame IDs:
*   `worker_1/base_link`
*   `worker_2/base_link`

This requires changing the **Robot State Publisher** configuration during launch.

## 4. Implementation: The Launch Logic

We use `PushRosNamespace` in our Python launch file.

```python
# code/module-2/launch/multi_warehouse.launch.py

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def spawn_robot(name, x, y):
    return GroupAction([
        PushRosNamespace(name), # Everything inside this block gets /name prefix
        
        # 1. State Publisher (Remaps frame_id)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': urdf,
                'frame_prefix': name + '/' # Crucial for TF!
            }]
        ),
        
        # 2. Spawn Entity (Gazebo)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', name, '-x', str(x), '-y', str(y), '-topic', 'robot_description']
        )
    ])

def generate_launch_description():
    return LaunchDescription([
        spawn_robot('robot_alpha', 0.0, 0.0),
        spawn_robot('robot_beta',  2.0, 0.0)
    ])
```

## 5. Engineering Insights: The Bridge Wildcard

We also need to bridge these new topics. Writing a YAML entry for every robot is tedious.
Modern `ros_gz_bridge` supports **Templates**.

```yaml
# bridge.yaml
- topic_name: "/<robot_name>/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```
*Note: Support for templates varies by ROS distribution. In Humble, you often script the YAML generation in Python before launching the bridge.*

## 6. Real-World Example: Swarm Formation

Research labs use this technique to test swarm algorithms. If you want to simulate a flock of 50 drones, you just loop `spawn_robot` 50 times.
The limiting factor is usually the CPU physics, not ROS.

## 7. Summary

We have successfully cloned our robot. We now have `robot_alpha` and `robot_beta` existing in the same physical space but operating on independent data channels.
In the next lesson, we will make things difficult for them by injecting faults.