---
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

<h2>2.1 Hardware Acceleration</h2>

Standard ROS nodes run on the CPU. Computer Vision (CV) algorithms are heavy and can choke the CPU.
**Isaac ROS** nodes use the GPU (via CUDA) to run CV tasks 10x-100x faster.

<h2>2.2 Visual SLAM (VSLAM)</h2>

**SLAM** (Simultaneous Localization and Mapping) answers two questions:
1.  Where am I? (Localization)
2.  What does the world look like? (Mapping)

**Visual SLAM** uses cameras instead of LiDAR. It tracks "features" (corners, edges) in the video feed to calculate movement.

<h2>2.3 Launching VSLAM</h2>

We use the `isaac_ros_visual_slam` package.

```python title="code/module-3/isaac_ros/visual_slam.launch.py"
import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Define the VSLAM Node
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[
            ('stereo_camera/left/image', '/camera/left/image_rect'),
            ('stereo_camera/right/image', '/camera/right/image_rect'),
            ('visual_slam/odom', '/odom')
        ],
        parameters=[{
            'enable_rectified_pose': True,
            'denoise_input_images': False,
            'rectified_images': True
        }]
    )

    # Run it in a Container (Standard for Isaac ROS)
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

<h2>2.4 Verification</h2>

1.  Ensure Isaac Sim is running and publishing stereo images.
2.  Launch the VSLAM node.
3.  Open **RViz2**.
4.  Visualize the `/odom` topic. Move the robot in Sim; the TF frame in RViz should move smoothly.