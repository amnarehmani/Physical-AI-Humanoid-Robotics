---
id: 03-lesson-3
title: "Navigation & Planning (Nav2)"
sidebar_label: "3. Nav2"
description: "Path planning for humanoid robots using the ROS 2 Nav2 stack."
keywords:
  - nav2
  - navigation
  - path planning
  - humanoid
  - ros2
---

# Lesson 3: Navigation & Planning (Nav2)

## 1. Introduction

We have a map. We know where we are. Now we need to move.
**Nav2** is the second generation of the ROS Navigation Stack. It is an industry-standard framework for autonomous navigation. It uses **Behavior Trees** to manage complex logic like "Plan path, if stuck, back up, if still stuck, spin."

For a humanoid robot, navigation is harder than for a Roomba. A humanoid is not a circle. It has a complex shape, and it might sway when walking.

## 2. Conceptual Understanding: The Costmap

Nav2 views the world as a grid of costs.
*   **0**: Free space (Safe).
*   **254**: Lethal obstacle (Wall).
*   **1-253**: Inflation zone (Too close to wall).

### Layers
1.  **Static Layer**: The map we built (Walls).
2.  **Obstacle Layer**: Real-time data from Lidar/Depth Camera (People, Boxes).
3.  **Inflation Layer**: Mathematical padding around obstacles so the robot doesn't scrape the paint.

## 3. System Perspective: The Nav2 Architecture

```text
      [ Application ] (e.g. "Go to Kitchen")
             |
             v
      [ BT Navigator ] (Behavior Tree Orchestrator)
             |
      +------+------+
      |             |
 [ Planner ]   [ Controller ]
 (Global Path) (Local Cmd_Vel)
      |             |
 [ Costmap ]   [ Costmap ]
```

## 4. Implementation: Configuring for Humanoids

We need to tell Nav2 that our robot is a rectangle (or polygon), not a circle. We do this in the `params.yaml` file.

Create `code/module-3/nav2/nav2_humanoid_params.yaml`.

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    # Localization parameters...

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    default_bt_xml_filename: "navigate_to_pose_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: True
    # Humanoid Footprint (Shoulders are wider than feet!)
    # Defined as [x, y] points relative to center
    footprint: "[ [0.2, 0.3], [0.2, -0.3], [-0.2, -0.3], [-0.2, 0.3] ]"
    
    # The "FollowPath" plugin (Pure Pursuit or MPC)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5 # Humanoids shouldn't run too fast indoors
      max_vel_theta: 1.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      global_frame: odom
      plugin_names: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        cost_scaling_factor: 3.0
        inflation_radius: 0.55 # Keep people at arm's length
```

## 5. Engineering Insights: Recovery Behaviors

What happens if the robot gets trapped?
Nav2 has **Recovery Behaviors**:
1.  **Spin**: Rotate in place to clear the costmap.
2.  **BackUp**: Reverse slowly.
3.  **Wait**: Pause and hope the obstacle (person) moves.

For a humanoid, "Spin" is dangerous (it might lose balance). We often disable aggressive recoveries or tune them to be very slow.

## 6. Real-World Example: The "Freezing Robot"

A common bug in deployment is the "Freezing Robot."
The robot sees an obstacle, stops, plans a new path, sees the obstacle again, stops... forever.
This is usually an issue with the **Local Costmap** not clearing. The robot thinks a person who walked away 5 minutes ago is still there.
**Fix**: Tune the `raytrace_range` and `obstacle_persistence` parameters in the costmap.

## 7. Summary

We have completed the "Brain" module.
1.  **Isaac Sim**: Provided the world.
2.  **Isaac ROS**: Provided the eyes (SLAM).
3.  **Nav2**: Provided the legs (Path Planning).

Our robot is now autonomous. It can accept a goal `(x=10, y=5)` and safely navigate there.
In Module 4, we will teach it to understand *why* it is going there.
