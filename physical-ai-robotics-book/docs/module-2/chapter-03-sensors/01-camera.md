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

## The Camera Link

First, add a link for the camera geometry (a small box) and join it to the robot.

```xml
<link name="camera_link">
  <visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.5" rpy="0 0 0"/>
</joint>
```

## The Gazebo Plugin

Now, we inject the sensor logic using the `<gazebo>` tag.

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="my_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>my_robot/camera1</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

## Key Parameters

*   **update_rate**: FPS (Frames Per Second). Higher = more CPU load.
*   **horizontal_fov**: Field of View in radians.
*   **width/height**: Resolution.
*   **plugin**: The bridge that takes Gazebo data and publishes ROS 2 topics (`/my_robot/camera1/image_raw`).

## Depth Cameras (RGB-D)

To simulate a Kinect or RealSense, change `type="camera"` to `type="depth"`.
The plugin filename becomes `libgazebo_ros_depth_camera.so`.
This publishes `sensor_msgs/PointCloud2` data, which allows the robot to see in 3D.

## End-of-Lesson Checklist

- [ ] I can add a camera sensor to my URDF.
- [ ] I can configure the resolution and FPS.
- [ ] I can view the simulated camera feed in Rviz2 (Add -> Image).
- [ ] I understand how to switch between standard RGB and Depth cameras.
