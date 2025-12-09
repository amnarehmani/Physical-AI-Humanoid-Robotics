---
id: m2-ch3-summary
title: "Chapter 3 Summary"
sidebar_label: "Summary & Quiz"
description: "Review of Sensors in Simulation."
keywords:
  - gazebo
  - summary
  - quiz
---

# Chapter 3 Summary

## Recap

In this chapter, we completed our Digital Twin.
*   We gave it **Sight** using Camera plugins (RGB and Depth).
*   We gave it **Spatial Awareness** using Lidar ray sensors.
*   We gave it **Balance** using IMU sensors.
*   We learned the critical lesson of **Sensor Noise**, ensuring our simulation prepares us for the messy reality of the physical world.

## Module 2 Conclusion

You have completed Module 2: The Digital Twin.
You now have a robot that:
1.  Looks real (Visual Mesh).
2.  Acts real (Inertia, Friction, Dynamics).
3.  Senses real (Cameras, Lidar, Noise).

This is a "high-fidelity" simulation. In **Module 3**, we will upgrade our brain. We will leave Gazebo (ROS 2 default) and enter **NVIDIA Isaac Sim**, a photorealistic simulator powered by RTX technology, designed for AI training.

## Mini Quiz

1.  **Which sensor type is used for Lidar in Gazebo?**
    *   *Answer: `ray`.*

2.  **What does an IMU measure?**
    *   *Answer: Linear acceleration and Angular velocity.*

3.  **Why do we add noise to simulation sensors?**
    *   *Answer: To mimic real hardware and prevent algorithms from overfitting to perfect data.*

4.  **What is the difference between `<visualize>true</visualize>` in Gazebo and viewing in Rviz?**
    *   *Answer: `visualize` draws debug lines inside the Gazebo window. Rviz subscribes to the actual ROS topic.*

5.  **Which plugin library handles the camera?**
    *   *Answer: `libgazebo_ros_camera.so` (or similar depending on version).*
