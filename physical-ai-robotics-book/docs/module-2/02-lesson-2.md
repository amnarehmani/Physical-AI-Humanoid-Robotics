---
id: 02-lesson-2
title: "Simulating Perception"
sidebar_label: "2. Sensors"
description: "Adding LiDAR, Camera, and IMU sensors to Gazebo simulation."
keywords:
  - sensors
  - lidar
  - camera
  - imu
  - plugins
  - gazebo
---

# Lesson 2: Simulating Perception (Sensors)

<h2>2.1 How Sensors Work in Simulation</h2>

Real sensors are messy. They have noise, dropout, and lens flares.
Simulated sensors are mathematical projections.
*   **LiDAR**: Raycasting. "Shoot 360 lines. Distance = Time * Speed of Light."
*   **Camera**: Rendering. "Draw the scene from this POV to a texture buffer."
*   **IMU**: Physics query. "What is the current acceleration vector of this link?"

To simulate these in Gazebo, we use **Plugins**.

<h2>2.2 Adding Plugins to URDF</h2>

Gazebo doesn't magically know a box is a camera. We must tell it by adding `<gazebo>` tags to our URDF/XACRO file.

Let's create a separate file `sensors.xacro` to keep our robot definition clean.

```xml title="code/module-2/urdf/sensors.xacro"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ========================================= -->
  <!-- 1. LiDAR Sensor Plugin                    -->
  <!-- ========================================= -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="gpu_lidar">
      <always_on>true</always_on>
      <visualize>true</visualize> <!-- Show blue rays in Gazebo -->
      <update_rate>10</update_rate> <!-- 10 Hz -->
      <topic>scan</topic> <!-- Publish to /scan -->
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14</min_angle> <!-- -180 deg -->
            <max_angle>3.14</max_angle>  <!-- +180 deg -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
        </range>
      </ray>
    </sensor>
  </gazebo>

  <!-- ========================================= -->
  <!-- 2. Camera Sensor Plugin                   -->
  <!-- ========================================= -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30</update_rate>
      <topic>camera/image_raw</topic>
      <camera>
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 deg FOV -->
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
    </sensor>
  </gazebo>

  <!-- ========================================= -->
  <!-- 3. IMU Sensor Plugin                      -->
  <!-- ========================================= -->
  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <topic>imu/data</topic>
    </sensor>
  </gazebo>

</robot>
```

<h2>2.3 Exercise: Verifying Data</h2>

Once you spawn a robot equipped with these plugins (by including `sensors.xacro` in your main URDF), Gazebo starts broadcasting.

1.  **Visualize LiDAR**: In Gazebo, you will see flickering blue lines coming from your robot.
2.  **Check Topics**:
    ```bash
    ros2 topic list
    ```
    *Expect:* `/scan`, `/camera/image_raw`, `/imu/data`.

3.  **Inspect Data**:
    ```bash
    ros2 topic echo /scan
    ```
    *Result:* A wall of numbers scrolling by. This is what the robot "sees".

:::info Note on Bridges
Remember the `ros_gz_bridge` from Lesson 1? You must ensure it is configured to bridge these specific topics, or ROS 2 won't hear Gazebo's shouting.
:::