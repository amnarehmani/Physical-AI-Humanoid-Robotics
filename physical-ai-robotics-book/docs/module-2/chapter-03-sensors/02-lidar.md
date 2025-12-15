---
id: m2-ch3-lidar
title: "Lesson 2: Simulating Lidar"
sidebar_label: "Lesson 2: Lidar"
description: "Adding 2D Laser Scanners for navigation."
keywords:
  - gazebo
  - lidar
  - ray
  - laserscan
---

# Lesson 2: Simulating Lidar

## 1. Introduction

Cameras are great, but they require heavy processing (Neural Networks) to understand geometry. **LiDAR** (Light Detection and Ranging) is direct. It tells you exactly how far away the wall is, with centimeter precision.

In robotics, LiDAR is the workhorse of SLAM (`Simultaneous Localization and Mapping`). If you want your robot to navigate a room without bumping into things, you start with a LiDAR.

## 2. Conceptual Understanding: Ray Casting

Simulated LiDAR is much simpler than a camera. It does not render pixels. It shoots **Rays**.

```text
      [ Ray Source ]
          /  |  \
         /   |   \   (Beams spread out)
        /    |    \
       X     X     X  (Intersection Points)
    [Wall] [Box] [Wall]
```

The physics engine calculates the geometric intersection of a line and a mesh.
*   **Time of Flight (Real)**: `d = (c * t) / 2`
*   **Ray Intersection (Sim)**: `d = || P_hit - P_origin ||`

## 3. System Perspective: The Scan Config

Gazebo defines a scan by its angular resolution and limits.

*   **Samples (`N`)**: Total number of rays per sweep (e.g., 360).
*   **Sweep Angle (`theta`)**: Total Field of View (e.g., `2*pi` or `360` degrees).
*   **Resolution**: Distance between rays.

`Angle Increment = (Sweep Angle) / (Samples - 1)`

## 4. Implementation: The URDF Config

We use the `type="ray"` sensor (or `gpu_ray` for GPU acceleration).

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize> <!-- Draws blue lines in GUI -->
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle> <!-- -180 deg -->
          <max_angle>3.14159</max_angle>  <!-- +180 deg -->
        </horizontal>
      </scan>
      <range>
        <min>0.10</min> <!-- Dead zone -->
        <max>12.0</max> <!-- Max range -->
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev> <!-- 1cm noise -->
      </noise>
    </ray>
    
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## 5. Engineering Insights: GPU vs CPU

Gazebo offers two types of ray sensors:
1.  **CPU Ray (`ray`)**: Uses the physics collision engine. Accurate but slow if you have complex meshes.
2.  **GPU Ray (`gpu_ray`)**: Uses the graphics card buffer. Much faster for high-res scans, but might miss "invisible" physics colliders.

**Recommendation**: Use `gpu_ray` if you simulate many robots or 3D LiDARs (`Velodyne/Ouster`). Use `ray` for simple 2D robots.

## 6. Real-World Example: The "Glass Wall" Problem

LiDARs cannot see glass. The light goes through it.
In Gazebo, if your glass wall has a `<collision>` geometry, the Ray sensor **will** see it (because rays hit collision shapes).
This is a `Sim-to-Real mismatch`.
To fix this, advanced users separate visual glass from collision glass, or use specific material properties, but often we just accept that simulation is "easier" than reality here.

## 7. Summary

We have added a `2D LiDAR` to our robot. It streams `sensor_msgs/LaserScan` data on the `/scan` topic. This array of distance measurements forms the foundation of the navigation stack we will build in Module 3.

Next, we will add the "Inner Ear" of the robot: the `IMU`.
