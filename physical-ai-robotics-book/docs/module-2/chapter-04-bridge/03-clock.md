---
id: m2-ch4-clock
title: "Lesson 3: Time Synchronization"
sidebar_label: "Lesson 3: Clock Sync"
description: "Handling Sim Time vs Real Time."
keywords:
  - use_sim_time
  - clock
  - synchronization
  - rtf
---

# Lesson 3: Time Synchronization

## The Relativity of Simulation

In the real world, 1 second is 1 second.
In simulation, 1 second might take 0.1 seconds to compute (fast) or 10 seconds to compute (slow/laggy). This is the **Real Time Factor (RTF)**.

If your navigation code assumes 1s has passed, but the simulator has only advanced 0.1s, your robot will crash.

## The `/clock` Topic

Gazebo publishes the current simulation time to the `/clock` topic.
All ROS 2 nodes must listen to this topic instead of reading the system wall clock.

## Enabling Sim Time

1.  **Bridge Configuration**: Ensure the bridge maps the clock.
    ```yaml
    - topic_name: "/clock"
      ros_type_name: "rosgraph_msgs/msg/Clock"
      gz_type_name: "gz.msgs.Clock"
      direction: GZ_TO_ROS
    ```

2.  **Node Configuration**: Set the parameter `use_sim_time=True` for **EVERY** node.

```python
Node(
    package='nav2_bringup',
    executable='bringup_launch',
    parameters=[{'use_sim_time': True}]
)
```

## What happens if you forget?

*   **TF Errors**: "Transform is too old" or "Transform is in the future."
*   **Controller Instability**: PID loops go crazy because `dt` (delta time) calculations are wrong.
*   **Bag Replay Fails**: If you play a bag with `use_sim_time=False`, nodes ignore the bag's timestamps.

## End-of-Lesson Checklist

- [ ] I understand the concept of Real Time Factor (RTF).
- [ ] I verified that `/clock` is being published (`ros2 topic hz /clock`).
- [ ] I know that `use_sim_time` must be set globally for the simulation to work correctly.
- [ ] I can recognize the symptoms of clock mismatch (TF errors).
