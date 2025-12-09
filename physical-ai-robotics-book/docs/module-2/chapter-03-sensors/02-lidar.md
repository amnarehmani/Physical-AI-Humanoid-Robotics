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

## The Ray Sensor

Lidar (Light Detection and Ranging) is the gold standard for 2D mapping and navigation. In Gazebo, this is simulated using a "Ray" sensor.

```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="head_hokuyo_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>40</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    
    <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Understanding the Scan

*   **samples**: How many laser beams are fired in one sweep (e.g., 720 points).
*   **min/max_angle**: The sweep width. +/- 1.57 rad is a 180-degree scan.
*   **range**: The distance the laser can see. 30m is typical for outdoor, 10m for indoor.
*   **visualize**: If true, Gazebo draws blue rays in the viewport (useful for debugging).

## The Output: LaserScan

The plugin publishes `sensor_msgs/LaserScan`.
*   **ranges**: An array of 720 floats (distances).
*   **intensities**: Return strength (optional).

Nav2 (ROS 2 Navigation Stack) consumes this message to build maps and avoid obstacles.

## End-of-Lesson Checklist

- [ ] I can attach a Ray sensor to my robot.
- [ ] I can adjust the scan angle (e.g., 180 vs 360 degrees).
- [ ] I can visualize the laser scan in Rviz2 (Add -> LaserScan).
- [ ] I know that `visualize=true` helps verify the sensor placement.
