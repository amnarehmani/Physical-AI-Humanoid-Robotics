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
---

# Lesson 3: Visualization with Rviz2

## Seeing What the Robot Sees

A terminal printing numbers is useful, but a 3D view is essential. **Rviz2** (ROS Visualization) is the primary tool for debugging spatial data.

## Launching Rviz2

```bash
ros2 run rviz2 rviz2
```

## The Interface

1.  **Displays Panel (Left)**: This is where you add data streams.
2.  **3D View (Center)**: The virtual world.
3.  **Global Options**: Set the "Fixed Frame". This is critical. It is usually `map` or `base_link`. If set incorrectly, nothing will appear.

## Adding Displays

Click "Add" in the bottom left. Common types:
*   **TF**: Visualizes the coordinate frames (XYZ axes) of the robot.
*   **RobotModel**: Loads the URDF (visual mesh) of the robot.
*   **LaserScan**: Shows Lidar dots.
*   **Image**: Shows camera feed.
*   **Marker / MarkerArray**: Shows custom shapes (arrows, cubes) published by your code.

## Publishing Markers

Markers are a great way to debug logic (e.g., "Where does the robot *think* the target is?").

```python
from visualization_msgs.msg import Marker

marker = Marker()
marker.header.frame_id = "base_link"
marker.type = Marker.SPHERE
marker.action = Marker.ADD
marker.pose.position.x = 1.0
marker.scale.x = 0.2
marker.scale.y = 0.2
marker.scale.z = 0.2
marker.color.a = 1.0 # Alpha (transparency)
marker.color.r = 1.0 # Red
publisher.publish(marker)
```
In Rviz, add a "Marker" display and subscribe to the topic. You will see a red sphere appear 1 meter in front of the robot origin.

## Saving Configuration

Setting up Rviz takes time. Always save your config: `File -> Save Config As -> my_robot.rviz`.
You can load this automatically in a Launch file:

```python
Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', 'path/to/my_robot.rviz']
)
```

## End-of-Lesson Checklist

- [ ] I can launch Rviz2 and set the correct Fixed Frame.
- [ ] I can visualize the TF tree inside Rviz2.
- [ ] I can publish a custom Marker from Python and see it in 3D.
- [ ] I have saved a `.rviz` configuration file.
