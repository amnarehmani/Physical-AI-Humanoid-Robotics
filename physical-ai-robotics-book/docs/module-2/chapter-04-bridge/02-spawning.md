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

## The Empty World

When you launch Gazebo, you often start with an empty world. You need to "Spawn" your robot into it.

## The `create` Node

The package `ros_gz_sim` provides a `create` node.

```bash
ros2 run ros_gz_sim create -topic /robot_description -name my_robot -z 0.5
```

*   `-topic`: Reads the URDF XML string from this topic.
*   `-name`: The name in the simulation tree.
*   `-z`: Spawn height (avoid clipping into the floor).

## Launch Integration

This is how we automate it in a launch file:

```python
# spawn_robot.launch.py
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Start Robot State Publisher (Publishes /robot_description)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}]
    )

    # 2. Spawn Entity
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-z', '0.1'
        ]
    )
    
    return LaunchDescription([rsp, spawn])
```

## Common Pitfalls

1.  **Robot Explodes**: You spawned it inside the ground (`-z 0`). Set `-z 0.2` to be safe.
2.  **Robot is Grey/White**: Textures didn't load. Check your mesh paths in the URDF (`package://` vs `file://`).
3.  **Nothing Happens**: Did you start the bridge? The `create` node might depend on a service call that needs the bridge active.

## End-of-Lesson Checklist

- [ ] I can use the `create` CLI tool to spawn a robot.
- [ ] I understand the dependency on `robot_state_publisher`.
- [ ] I have integrated the spawn action into a Python launch file.
- [ ] I know how to debug a failed spawn (check console logs).
