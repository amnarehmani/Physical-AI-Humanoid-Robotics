---
id: m2-ch3-imu
title: "Lesson 3: IMU and Noise"
sidebar_label: "Lesson 3: IMU & Noise"
description: "Simulating accelerometers, gyros, and sensor noise."
keywords:
  - gazebo
  - imu
  - noise
  - gaussian
---

# Lesson 3: IMU and Noise

## 1. Introduction

How do you know which way is up? How do you know if you are falling?
For humans, it's the inner ear (vestibular system). For robots, it's the **IMU** (`Inertial Measurement Unit`).

The IMU is unique. It doesn't look at the world; it looks at the robot itself. It measures:
1.  **Linear Acceleration** (`m/s^2`): Including Gravity (`9.81 m/s^2`).
2.  **Angular Velocity** (`rad/s`): How fast it is spinning.

For a humanoid robot, the IMU is the most critical sensor for balance. If the IMU lags by `10ms`, the robot falls.

## 2. Conceptual Understanding: Physics Query

Simulating an IMU is mathematically trivial compared to a Camera. The simulator already knows the exact state of every link (`x_vec`, `v_vec`, `a_vec`).
The IMU plugin simply asks the physics engine: "What is the acceleration vector of `base_link` right now?"

`a_measured_vec = a_true_vec + g_vec + noise_vec + bias_vec`

Note that an accelerometer measures **Proper Acceleration**. A robot in freefall measures `a_vec = 0`. A robot sitting on a table measures `a_vec = 9.81` (upwards).

## 3. System Perspective: Noise Models

Perfect IMUs don't exist. Real MEMS sensors have:
*   **White Noise**: Random jitter.
*   **Bias**: A slow drift over time (`Random Walk`).

If you integrate a perfect IMU, you get perfect position. If you integrate a real IMU, you get lost in seconds.

```text
      [ True Velocity ]
             |
             v
      [ + Bias (Drift) ] --> (Slowly changing error)
             |
             v
      [ + Gaussian Noise ] --> (High frequency fuzz)
             |
             v
      [ Measured Velocity ]
```

## 4. Implementation: The URDF Config

We attach the IMU to the `imu_link` (usually near the center of mass).

```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate> <!-- Fast! -->
    <visualize>true</visualize>
    
    <imu>
      <angular_velocity>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
        <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
        <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
      </linear_acceleration>
    </imu>

    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <remapping>~/out:=imu</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## 5. Engineering Insights: Update Rates

Why did we set `update_rate` to 100 Hz?
*   **Cameras**: 30 Hz is fine.
*   **LiDAR**: 10 Hz is fine.
*   **IMU**: `100-1000 Hz` is required.

Because we **integrate** IMU data to get velocity and position (`v = integral of a * dt`), errors accumulate with time. A faster update rate reduces integration error (`Delta t` is smaller).
Simulating an IMU at 10 Hz is useless for balancing.

## 6. Real-World Example: Sensor Fusion

Since IMUs drift and LiDARs are slow, we combine them.
*   **IMU**: Gives fast, smooth updates (100 Hz) but drifts.
*   **LiDAR**: Gives slow, accurate updates (10 Hz) but corrects drift.

This combination is called **Sensor Fusion** (often using an `Extended Kalman Filter` or `EKF`). In simulation, having realistic noise on both sensors is the *only* way to verify your EKF works.

## 7. Summary

We have equipped our robot with the "Holy Trinity" of sensors:
1.  **Camera** (Vision)
2.  **LiDAR** (Geometry)
3.  **IMU** (Balance/Proprioception)

Our Digital Twin is now complete. It looks like a robot, acts like a robot, and senses like a robot. In the final summary, we will review the entire system before moving to Module 3.
