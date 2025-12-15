---
id: m2-ch3-camera
title: "Lesson 1: Simulating Cameras"
sidebar_label: "Lesson 1: Cameras"
description: "Adding RGB and Depth cameras to URDF."
keywords:
  - gazebo
  - camera
  - depth
  - pointcloud
---

# Lesson 1: Simulating Cameras

## 1. Introduction

Computer Vision is the dominant sensing modality in modern robotics. Whether it's a Tesla detecting lanes or a Roomba avoiding socks, cameras are everywhere.

Simulating a camera involves **Rendering**. Gazebo must pause the physics engine, switch to the graphics engine (OGRE), draw the scene from the camera's perspective, and then dump those pixels into a memory buffer. This is expensive. A poorly configured camera simulation can drop your simulation speed from 1000 Hz to 10 Hz.

## 2. Conceptual Understanding: The Pinhole Model

Gazebo uses the **Pinhole Camera Model** approximation.

```text
      Object (3D)
          |
          v
      [ Lens ]  <-- (Focal Length)
          |
          v
      [ Sensor ] <-- (Resolution W x H)
```

Key Parameters:
*   **Horizontal FOV (Field of View)**: How "wide" the camera sees.
    *   Human Eye: ~2.0 radians (~120 deg).
    *   Telephoto: ~0.5 radians.
    *   Wide Angle: ~1.5 radians.
*   **Clip Planes**: The "Near" and "Far" limits.
    *   `near`: Objects closer than this are invisible (clipping).
    *   `far`: Objects further than this are not rendered (fog).

## 3. System Perspective: The Render Pipeline

```text
      Gazebo Render Loop
             |
             v
      [ Render Frame ] --> (GPU Buffer)
             |
             v
      [ Apply Noise ]  --> (Gaussian Pixel Noise)
             |
             v
      [ Apply Distortion ] --> (Lens warping)
             |
             v
      [ Serialize ROS Msg ] --> /camera/image_raw
```

## 4. Implementation: The URDF Code

### 4.1 The Physical Link
Cameras are physical objects. We need a link.

```xml
<link name="camera_link">
  <visual>
    <geometry><box size="0.05 0.05 0.05"/></geometry>
    <material name="black"/>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.1" rpy="0 0 0"/> <!-- Facing Forward -->
</joint>
```

### 4.2 The Sensor Plugin
We inject the "Soul" of the camera using the `<gazebo>` tag.

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="main_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.396</horizontal_fov> <!-- ~80 deg -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev> <!-- Grainy film effect -->
      </noise>
    </camera>
    
    <!-- The ROS 2 Bridge -->
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>my_robot</namespace>
        <remapping>image_raw:=camera/image</remapping>
        <remapping>camera_info:=camera/camera_info</remapping>
      </ros>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## 5. Depth Cameras (RGB-D)

To simulate a Kinect or RealSense, we use `type="depth"`.
This produces an extra topic: **Depth Map**.
A depth map is a greyscale image where `White = Close` and `Black = Far` (or vice versa depending on encoding). This allows the robot to generate a 3D PointCloud.

## 6. Engineering Insights: Distortion

Real cameras have **Lens Distortion** (fisheye effect). Simulation cameras are perfect rectilinears.
If you train an AI on undistorted simulation images, it will fail on distorted real images.
**Best Practice**: Either undistort your real images (using OpenCV calibration) or add distortion to your simulation using the `<distortion>` tag in Gazebo.

## 7. Summary

We have given our robot eyes. It can now publish:
1.  **RGB Images**: For object detection and human viewing.
2.  **Depth Images**: For 3D perception and obstacle avoidance.

However, cameras are heavy on bandwidth and sensitive to light. For reliable geometry detection (like walls), we often use a different sensor: the **LiDAR**.