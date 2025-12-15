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
  - kinematics
---

# Lesson 2: TF2 and Transforms

## 1. Introduction

In the previous lesson, we defined *what* data to send. Now we must define *where* that data came from.

Robots don't just exist in code; they exist in geometry.
*   The camera is 0.5m above the floor.
*   The arm is 0.2m to the right of the camera.
*   The cup is 1.0m in front of the camera.

If you tell the arm to "Grab the cup at (1.0, 0.0, 0.0)," the arm will grasp at empty air because (1.0, 0.0, 0.0) relative to the **Camera** is very different from (1.0, 0.0, 0.0) relative to the **Shoulder**.

This lesson introduces **TF2** (The Transform Library v2), the standard way ROS 2 manages the complex web of moving coordinate frames that make up a robot.

## 2. Conceptual Understanding: The Relativity of Space

**Intuition**:
Imagine you are sitting on a train.
*   To **you**, the coffee cup on your tray table is stationary (`velocity = 0`).
*   To an **observer on the platform**, that same cup is moving at 100 km/h.

Neither is wrong. They just have different **Reference Frames**.
To convert between them, you need a **Transform** (the position and velocity of the train relative to the platform).

**In Robotics**:
*   **Lidar Frame**: Reports obstacle at `x=5.0` (5 meters ahead of the sensor).
*   **Base Link Frame**: If the lidar is mounted on the *back* of the robot facing backwards, that obstacle is actually at `x=-5.0` (5 meters *behind* the robot center).

## 3. System Perspective: The TF Tree

TF2 organizes the world into a single, directed acyclic graph (tree) of coordinate frames.

```mermaid-text
        [map] (World Fixed Frame)
          |
          v (Dynamic Transform: Localization)
        [odom] (Odometry Frame)
          |
          v (Dynamic Transform: Physics)
     [base_link] (The Robot Chassis)
          |
          +-----------------------------+
          |                             |
          v (Static Transform)          v (Dynamic Transform)
    [camera_link]                  [shoulder_link]
          |                             |
          v (Optical Transform)         v (Joint State)
    [camera_optical_frame]         [elbow_link]
```

1.  **Map -> Odom**: Corrects for drift (SLAM).
2.  **Odom -> Base**: The robot moving in the world.
3.  **Base -> Sensors**: Fixed mounting points (Bolts).
4.  **Base -> Limbs**: Moving joints (Motors).

## 4. Practical Example: The Listener

We will write a node that answers the question: "Where is the `camera_link` right now, relative to the `map`?"

### 4.1 The Code

```python title="code/module-1/tf_listener.py"
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        
        # 1. Create a Buffer (Stores history of transforms)
        self.tf_buffer = Buffer()
        
        # 2. Create a Listener (Subscribes to /tf and fills buffer)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        try:
            # 3. Lookup Transform
            # Target: 'map' (We want coordinates in this frame)
            # Source: 'camera_link' (The point we are transforming)
            # Time: Time() (Zero means "latest available")
            t = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                rclpy.time.Time())
                
            self.get_logger().info(
                f'Camera is at: x={t.transform.translation.x:.2f}, y={t.transform.translation.y:.2f}')
                
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')

def main():
    rclpy.init()
    node = TfListener()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 4.2 Publishing a Static Transform
To test this without a real robot simulation, we can fake the camera position:

```bash
# syntax: x y z yaw pitch roll parent child
ros2 run tf2_ros static_transform_publisher 1 2 0 0 0 0 map camera_link
```

## 5. Engineering Insights: Static vs Dynamic

*   **Static Transforms (`/tf_static`)**:
    *   What: Transforms that rarely change (e.g., Camera bolted to chassis).
    *   Optimization: Sent **once** (latched). Saves massive bandwidth.
    *   Tool: `static_transform_publisher`.
*   **Dynamic Transforms (`/tf`)**:
    *   What: Transforms that change constantly (e.g., Wheels turning, Arm moving).
    *   Optimization: Sent at high frequency (50Hz+).
    *   Source: Typically `robot_state_publisher` (which reads URDF and Joint States).

**Common Pitfall**:
Never publish a static transform on the dynamic `/tf` topic at 100Hz. It floods the network with redundant data.

## 6. Summary

TF2 is the glue that holds the robot's geometry together.
1.  **Frames** are coordinate systems attached to body parts.
2.  **Transforms** define the relationship between parents and children.
3.  **Buffers** store the history, allowing us to ask "Where was the hand 1 second ago?"

With Custom Interfaces (data) and TF2 (space), we have the complete picture. The final step is to verify everything visually. In the next lesson, we will use **Rviz2** to see what the robot sees.