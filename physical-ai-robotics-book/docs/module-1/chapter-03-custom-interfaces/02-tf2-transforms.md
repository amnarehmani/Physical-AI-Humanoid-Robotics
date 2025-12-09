---
id: m1-ch3-tf2
title: "Lesson 2: TF2 and Transforms"
sidebar_label: "Lesson 2: TF2 Transforms"
description: "Understanding the Transform Tree and coordinate frames."
keywords:
  - ros2
  - tf2
  - transforms
  - frames
  - rviz
---

# Lesson 2: TF2 and Transforms

## The Coordinate Frame Problem

A Lidar sensor reports an obstacle at `x=2.0, y=0.0`. This is relative to the *Lidar*.
If the Lidar is mounted on the back of the robot facing backwards, that obstacle is actually *behind* the robot.
If the robot is rotated 90 degrees in the room, the obstacle is at a completely different world coordinate.

Manually doing this trigonometry (Rotation matrices, Translation vectors) is painful and error-prone. **TF2** does it for you.

## The TF Tree

TF2 organizes frames in a tree structure.
*   **Root**: Usually `map` or `world`.
*   **Children**: `odom` -> `base_link` -> `sensor_link`.

Each connection (edge) in the tree is a **Transform** (Translation + Rotation).
Some transforms are **Static** (the sensor is bolted to the chassis).
Some are **Dynamic** (the robot moves relative to the map, or an arm joint rotates).

## Publishing a Static Transform

If you have a sensor mounted on your robot, you need to tell the system where it is.
We use the `static_transform_publisher`.

```bash
# syntax: x y z yaw pitch roll parent_frame child_frame
ros2 run tf2_ros static_transform_publisher 0.5 0 0.2 0 0 0 base_link camera_link
```
This says: "The `camera_link` is 0.5m forward and 0.2m up from `base_link`."

## Listening to Transforms in Python

To convert data, we use a **TF Buffer** and **Listener**.

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            # Look up transform from 'base_link' to 'map'
            # "Where is the base_link expressed in map coordinates?"
            t = self.tf_buffer.lookup_transform(
                'map',          # Target frame
                'base_link',    # Source frame
                rclpy.time.Time()) # Time (0 means latest)
                
            self.get_logger().info(
                f'Robot is at: {t.transform.translation.x}, {t.transform.translation.y}')
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
```

## Debugging TF

The `tf2_tools` package provides `view_frames`:

```bash
ros2 run tf2_tools view_frames
```
This generates a PDF showing the current tree. A broken tree (disconnected parts) is a common error source.

## End-of-Lesson Checklist

- [ ] I understand the parent-child relationship in a TF tree.
- [ ] I have used `static_transform_publisher` to link two frames.
- [ ] I can write a listener to look up the position of one frame relative to another.
- [ ] I know how to check if the TF tree is connected.
