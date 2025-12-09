---
id: m2-ch4-summary
title: "Chapter 4 Summary"
sidebar_label: "Summary & Quiz"
description: "Review of the ROS-Gazebo Bridge."
keywords:
  - summary
  - quiz
  - bridge
---

# Chapter 4 Summary

## Recap

In this chapter, we connected the brain to the body.
*   We used **ros_gz_bridge** to create a wormhole between ROS 2 topics and Gazebo topics.
*   We used **ros_gz_sim** to spawn our URDF robot into the world programmatically.
*   We synchronized the **Time** dimensions of both worlds using the `/clock` topic, ensuring that our control algorithms respect the physics engine's processing speed.

## Future Outlook

Now that we have a connected robot, we need a world for it to live in. In the final chapter of Module 2, we will build a **Warehouse Digital Twin**, complete with shelves, obstacles, and multiple robots.

## Mini Quiz

1.  **What is the purpose of the `ros_gz_bridge`?**
    *   *Answer: To translate messages between ROS 2 (DDS) and Gazebo (Ignition Transport).*

2.  **Which direction should a Lidar topic be mapped?**
    *   *Answer: GZ_TO_ROS (Sensor -> Node).*

3.  **If you spawn a robot and it immediately falls through the floor, what is likely wrong?**
    *   *Answer: The spawn height (`-z`) was too low, or the collision geometry is missing.*

4.  **What parameter makes a ROS node listen to the simulation clock?**
    *   *Answer: `use_sim_time`.*

5.  **Can you run the bridge without a config file?**
    *   *Answer: Yes, by passing arguments via CLI, but YAML is preferred for complex setups.*
