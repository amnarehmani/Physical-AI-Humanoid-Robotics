---
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

<h2>3.1 The Navigation Stack</h2>

Knowing where you are (SLAM) is step 1. Deciding where to go and how to get there is step 2.
**Nav2** is the standard navigation stack for ROS 2. It handles:
*   **Global Planning**: Finding a path on the map (A* algorithm).
*   **Local Planning**: Avoiding dynamic obstacles (DWB Controller).
*   **Recovery**: What to do if stuck (Spin, Back up).

<h2>3.2 Configuring for Humanoids</h2>

Most robots are circles (Roombas). Humanoids are not. We must define a custom **footprint**.

```yaml title="code/module-3/nav2/nav2_humanoid_params.yaml"
amcl:
  ros__parameters:
    use_sim_time: True
    # ... AMCL settings ...

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
    # Define the robot shape (Polygon)
    # Example: A rectangle 0.5m wide, 0.3m deep
    footprint: "[ [0.15, 0.25], [0.15, -0.25], [-0.15, -0.25], [-0.15, 0.25] ]"
    controller_plugins: ["FollowPath"]

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      global_frame: odom
      plugin_names: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        # Inflate obstacles by 0.55m (radius + safety)
        inflation_radius: 0.55 
```

<h2>3.3 Launching Navigation</h2>

```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=code/module-3/nav2/nav2_humanoid_params.yaml
```

<h2>3.4 Exercise: Point-and-Click</h2>

1.  Launch Isaac Sim + VSLAM + Nav2.
2.  Open RViz2.
3.  Use the "2D Nav Goal" tool to click a point on the map.
4.  Watch the robot plan a green path and follow it!