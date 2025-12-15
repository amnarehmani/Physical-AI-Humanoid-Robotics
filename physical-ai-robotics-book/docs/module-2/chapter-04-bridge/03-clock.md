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

## 1. Introduction

Time is relative. Einstein proved it for light speed; Gazebo proves it for CPU speed.

In the real world, 1 second is always 1 second.
In simulation, "1 second" is a slice of computed physics steps.
*   **Fast Computer**: The simulation might run at 5x speed (1 sim-second happens in 0.2 real-seconds).
*   **Slow Computer**: The simulation might run at 0.1x speed (1 sim-second takes 10 real-seconds).

If your ROS nodes read the **Wall Clock** (System Time), but the robot lives in **Sim Time**, chaos ensues. Navigation timeouts trigger, PIDs diverge, and TF transforms expire.

## 2. Conceptual Understanding: The Clock Server

We solve this by designating Gazebo as the **Time Lord**.
1.  Gazebo calculates a physics step ($t + \Delta t$).
2.  Gazebo publishes the new time `t` to the `/clock` topic.
3.  All ROS nodes subscribe to `/clock` and pause their internal timers until they receive an update.

This ensures that even if the simulation freezes, the ROS nodes "freeze" with it.

## 3. System Perspective: The `use_sim_time` Parameter

Every standard ROS 2 node checks a parameter called `use_sim_time`.

*   `False` (Default): `node.get_clock().now()` returns System Time.
*   `True`: `node.get_clock().now()` returns the last value received on `/clock`.

```text
      [ Gazebo ] --(/clock)--> [ ROS Bridge ] --(/clock)--> [ Navigation Node ]
                                                                  ^
                                                                  |
                                                          (use_sim_time=True)
```

## 4. Implementation: Configuring the Clock

### 4.1 Bridge Config
The bridge must forward the clock message. Most default launch files do this automatically, but if you write your own YAML:

```yaml
- topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

### 4.2 Launch File Config
When launching nodes, you must pass the parameter.

```python
# navigation.launch.py
Node(
    package='nav2_bringup',
    executable='bringup_launch',
    parameters=[{'use_sim_time': True}]  # <--- CRITICAL
)
```

## 5. Engineering Insights: TF Errors

The #1 symptom of clock mismatch is **TF Errors**:
*   *"Lookup would require extrapolation into the future"*
*   *"Lookup would require extrapolation into the past"*

This happens because the TF Tree (robot posture) is stamped with Sim Time (e.g., `t=50`), but your node is reading Wall Time (e.g., `t=1734259800`). The difference is 50 years. The TF buffer drops the packet immediately.

**Debug Command**:
```bash
ros2 param get /my_node use_sim_time
# Should return "Boolean value is: True"
```

## 6. Real-World Example: Fast-Forward Training

In Reinforcement Learning (RL), we want to train fast.
If we set `use_sim_time=True` and uncap the Gazebo physics speed (RTF > 1.0), we can train a robot 100x faster than real life.
The ROS nodes don't care. They just see the clock ticking very fast. The logic holds up perfectly. This is the superpower of Simulation.

## 7. Summary

Time synchronization is the invisible glue of simulation.
*   **Real Time Factor (RTF)**: The ratio of Sim Time to Wall Time.
*   **`/clock`**: The heartbeat of the simulation.
*   **`use_sim_time`**: The switch that tells ROS to listen to the heartbeat.

We have now completed the infrastructure. We have a Bridge, we can Spawn, and we are Synchronized.
In the final summary, we will verify the entire "ROS-Gazebo" ecosystem.