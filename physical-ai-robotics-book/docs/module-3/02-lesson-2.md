---
id: 02-lesson-2
title: "Accelerated Perception (Isaac ROS)"
sidebar_label: "2. Isaac ROS (VSLAM)"
description: "Using hardware acceleration for Visual SLAM with Isaac ROS."
keywords:
  - isaac ros
  - vslam
  - gpu
  - perception
  - slam
---

# Lesson 2: Accelerated Perception (Isaac ROS)

## 1. Introduction

In traditional robotics, the CPU does everything: planning, control, and perception. This is fine for a Roomba, but it fails for a humanoid. Processing two 4K camera streams at 60 FPS to calculate depth and pose will saturate even an i9 CPU in milliseconds.

**Isaac ROS** solves this by offloading perception tasks to the GPU. It uses **NITROS** (NVIDIA Isaac Transport for ROS), a middleware that allows nodes to pass pointers to GPU memory instead of copying data.

In this lesson, we will implement **Visual SLAM (VSLAM)**. We will use the robot's cameras to figure out where it is, without needing LiDAR.

## 2. Conceptual Understanding: Visual SLAM

SLAM stands for **Simultaneous Localization and Mapping**.
*   **Visual Odometry**: Comparing Frame $t$ with Frame $t-1$ to calculate $\Delta x$.
*   **Loop Closure**: Realizing "I have been here before" and correcting drift.

### The Isaac ROS Pipeline

```text
      [ Camera Source ]
      (Isaac Sim / RealSense)
             |
             v
      [ Image Rectifier ]  <-- (GPU Warp)
             |
             v
      [ Visual SLAM Node ] <-- (GPU Feature Tracking)
             |
             v
      [ /visual_slam/odom ] --> (To Nav2)
```

The data stays on the GPU. We only get the result (Position `x,y,z`) back on the CPU.

## 3. System Perspective: Docker Containers

Isaac ROS libraries rely on specific CUDA, TensorRT, and Ubuntu versions. Installing them natively is a nightmare.
We **always** run Isaac ROS inside a Docker container provided by NVIDIA.

```bash
# Standard workflow
cd ${ISAAC_ROS_WS}/src/isaac_ros_common
./scripts/run_dev.sh
```

Inside this container, you have access to the GPU and all pre-compiled libraries.

## 4. Implementation: The VSLAM Launch File

Create `code/module-3/isaac_ros/visual_slam.launch.py`.

We use a **Composable Node**. This allows multiple nodes (Rectification + SLAM) to share the same process and GPU context, enabling zero-copy transfer.

```python
import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Define the Visual SLAM Node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[
            # Input: Stereo Images
            ('stereo_camera/left/image', '/camera/left/image_raw'),
            ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
            ('stereo_camera/right/image', '/camera/right/image_raw'),
            ('stereo_camera/right/camera_info', '/camera/right/camera_info'),
            # Output: The calculated position
            ('visual_slam/odom', '/odom')
        ],
        parameters=[{
            'enable_rectified_pose': True,
            'denoise_input_images': False,
            'rectified_images': True,
            'enable_imu_fusion': True, # Use IMU to help tracking
            'gyro_noise_density': 0.000244, # Parameters match simulation noise
            'accel_noise_density': 0.001862
        }]
    )

    # 2. Run it in a Container
    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([container])
```

## 5. Engineering Insights: Feature Descriptors

How does VSLAM track movement? It looks for "interesting" points (corners, high contrast patches) called **Features**.
Isaac ROS uses GPU-accelerated feature extraction (like ORB or FAST).
**Warning**: VSLAM fails in feature-less environments (e.g., white walls). If your robot gets lost in a hallway, add posters or texture to the walls.

## 6. Real-World Example: Warehouse Localization

In a warehouse, GPS doesn't work (indoor). LiDAR can get confused by long, identical aisles.
VSLAM looks at the ceiling lights and floor patterns. It creates a "Visual Map" that is robust to changes in the shelf contents. Amazon robots use downward-facing cameras to read QR codes on the floor for absolute localization, fusing it with VSLAM.

## 7. Summary

We have offloaded the most computationally expensive task in robotics—SLAM—to the GPU.
*   **Input**: Stereo Images + IMU.
*   **Output**: Odometry (Position/Velocity).

With a reliable pose estimate, we can now answer the question: "How do I get to the destination?"
In the next lesson, we will feed this Odometry into **Nav2**.
