---
id: m1-ch3-rviz
title: "Lesson 3: Visualization with Rviz2"
sidebar_label: "Lesson 3: Rviz2"
description: "Visualizing robot state, sensors, and transforms."
keywords:
  - ros2
  - rviz2
  - visualization
  - gui
  - markers
---

# Lesson 3: Visualization with Rviz2

## 1. Introduction

A robot is a complex system of invisible numbers.
*   The arm is at angle `1.57`.
*   The lidar sees an obstacle at `3.2 meters`.
*   The path planner has found a trajectory.

If you rely on `print()` statements to debug this, you will fail. You cannot mentally reconstruct 3D geometry from a scrolling wall of text.

**Rviz2** (ROS Visualization) is the window into the robot's mind. It renders the robot's geometric understanding of the world. It is not a simulator (it doesn't calculate physics); it is a visualizer (it draws what the robot *thinks* is happening).

## 2. Conceptual Understanding: The Debugger

**Intuition**:
Think of Rviz2 as the **HTML Inspector** for robotics.
*   In web dev, you inspect the DOM to see why a button is misaligned.
*   In robotics, you inspect Rviz to see why the robot thinks the door is 1 meter to the left.

**Mechanism**:
Rviz2 is a massive subscriber.
1.  It subscribes to `/tf` to know where the body parts are.
2.  It subscribes to `/scan` to draw laser dots.
3.  It subscribes to `/camera/image` to show video.
4.  It renders all of this in a unified 3D OpenGL scene.

## 3. System Perspective: The Rendering Pipeline

```mermaid-text
[Hardware / Simulation]
       |
       v
(1) Sensor Data (Lidar, Camera) ---> [Topic: /scan]
       |
       v
(2) State Data (Joints, Odometry) -> [Topic: /tf]
       |
       v
[Rviz2 Application]
       |
       +---> [Display: LaserScan] (Draws dots based on /tf + /scan)
       |
       +---> [Display: RobotModel] (Draws mesh based on URDF + /tf)
       |
       +---> [Display: Camera] (Draws texture based on /image_raw)
```

## 4. Practical Example: Debugging with Markers

Sometimes standard displays (Laser, Image) aren't enough. You want to see the robot's *decisions*. "Where is the target point?" "What is the planned path?"

We use **Markers** to draw arbitrary 3D shapes.

### 4.1 The Marker Publisher

```python title="code/module-1/marker_publisher.py"
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MarkerDemo(Node):
    def __init__(self):
        super().__init__('marker_demo')
        self.pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)

    def publish_marker(self):
        marker = Marker()
        # 1. Header: Where and When
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # 2. Namespace and ID (Unique identifier)
        marker.ns = "target_point"
        marker.id = 0
        
        # 3. Shape and Action
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # 4. Pose (Relative to base_link)
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.5
        marker.pose.position.z = 0.2
        
        # 5. Scale (Meters)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # 6. Color (RGBA, 0-1)
        marker.color.a = 1.0 # Don't forget alpha!
        marker.color.r = 1.0 # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.pub.publish(marker)

def main():
    rclpy.init()
    node = MarkerDemo()
    rclpy.spin(node)
    rclpy.shutdown()
```

## 5. Engineering Insights: Rviz Survival Guide

*   **The "Fixed Frame" Error**: The most common Rviz error is "Transform [sensor] to [map] does not exist."
    *   *Cause*: Rviz needs a stationary anchor.
    *   *Fix*: Set "Fixed Frame" (Global Options) to `odom` or `map`. If testing a stationary robot, `base_link` works.
*   **Topic vs. Transform**:
    *   *Symptom*: You see the Lidar topic, but no dots appear.
    *   *Cause*: Rviz has the data, but doesn't know *where* to draw it because the TF link is broken.
    *   *Check*: Look at the "TF" display status.
*   **Saving Configs**: Configuring Rviz takes 5 minutes. Save your work (`File -> Save Config As`) into your package's `rviz/` folder. Load it automatically in launch files:
    ```python
    arguments=['-d', os.path.join(pkg_dir, 'rviz', 'debug.rviz')]
    ```

## 6. Summary

Rviz2 transforms data into insight.
1.  **Displays** render topics.
2.  **Markers** render logic.
3.  **TF** binds it all together in a coherent spatial tree.

This concludes Chapter 3. You can now Define data (Interfaces), Transform it (TF2), and See it (Rviz2). You have completed the Core Fundamentals of ROS 2.