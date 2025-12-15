---
id: m1-ch4-bag
title: "Lesson 1: Time Travel with Rosbag"
sidebar_label: "Lesson 1: Rosbag"
description: "Recording and replaying topic data."
keywords:
  - ros2
  - rosbag
  - recording
  - playback
  - mcap
  - sqlite3
---

# Lesson 1: Time Travel with Rosbag

## 1. Introduction

In traditional software debugging, you use a debugger to pause execution. But you cannot pause a falling robot. You cannot pause a moving car to check a variable. Reality runs at 1x speed.

To debug reality, we need a way to capture it. **Rosbag** is the "Time Machine" of ROS 2. It allows you to record the state of the world (Topics) and replay it later, fooling your nodes into thinking they are back in the past.

## 2. Conceptual Understanding: The Flight Recorder

**Intuition**:
Think of a **Black Box** on an airplane.
*   **Recording**: It passively listens to every sensor (Altitude, Speed, Cockpit Audio). It does not interfere with the pilot.
*   **Replay**: After an incident, investigators replay the data to reconstruct the event.

**Mechanism**:
*   `ros2 bag record`: Creates a new node that subscribes to user-defined topics. It serializes incoming messages to disk (SQLite3 or MCAP format).
*   `ros2 bag play`: Opens the file, deserializes messages, and publishes them on the original topics, respecting the original time delays.

## 3. System Perspective: The Simulation Loop

Rosbag enables **Simulation-Based Development** using real-world data.

```mermaid-text
       [Real Robot]
            |
            v
    (1) Record /camera/rgb
            |
            v
      [bag_file.mcap]
            |
            +---------------------------------+
            |                                 |
            v                                 v
(2) Play /camera/rgb              (3) Play /camera/rgb
    (Developer Laptop)                (CI/CD Server)
            |                                 |
            v                                 v
   [Face_Detector_v1]                [Face_Detector_v2]
```

1.  **Field Test**: Go outside once. Record the difficult lighting condition.
2.  **Dev Cycle**: Replay that specific difficult moment 100 times until your code handles it perfectly.
3.  **Regression Testing**: Add this bag to your Continuous Integration suite to ensure you never break this fix.

## 4. Practical Example: Recording Reality

### Step 1: Record
Start your sensors (or a simulation). Then, record specific topics.
```bash
# Syntax: ros2 bag record -o <folder_name> <topics...>
ros2 bag record -o my_test_data /image_raw /scan /odom
```
*   `-o`: Sets the output directory name.
*   `-a`: Records **ALL** topics. **Warning**: If you have raw camera feeds, this will fill your hard drive in minutes.

### Step 2: Info
Inspect what you captured.
```bash
ros2 bag info my_test_data
```
**Output**:
```text
Files:             my_test_data.mcap
Duration:          12.5s
Start:             Dec 13 2025 14:00:00
Messages:          1500
Topic information:
  Topic: /image_raw | Type: sensor_msgs/msg/Image | Count: 375
  Topic: /scan      | Type: sensor_msgs/msg/LaserScan | Count: 125
```

### Step 3: Replay
Fool your system.
```bash
ros2 bag play my_test_data --loop --rate 0.5
```
*   `--loop`: Play continuously (great for tuning visualization).
*   `--rate 0.5`: Play in slow motion (great for debugging fast events).

## 5. Engineering Insights: Storage & Formats

*   **MCAP vs SQLite3**:
    *   **SQLite3**: The old default. Compatible with SQL tools. Slower for high-bandwidth data.
    *   **MCAP (Standard since Iron)**: A file format optimized specifically for robotics streaming. It handles writing multiple GB/s much better than SQLite. **Always use MCAP.**
*   **Disk Usage**:
    *   Lidar/IMU: Negligible (~KB/s).
    *   Video (1080p, 30fps): ~100-300 MB/s per camera.
    *   *Tip*: Use `ros2 bag record --compression-mode message --compression-format zstd` to compress data on the fly if your CPU is strong but disk is slow.

## 6. Summary

Rosbag is the bridge between the chaos of the real world and the controlled environment of your IDE.
1.  **Record** problems in the field.
2.  **Replay** them in the lab.
3.  **Fix** the code.

Now that we can record data, we need tools to inspect the live system when things go wrong. In the next lesson, we will meet the system doctor.