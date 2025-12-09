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
  - sqlite3
---

# Lesson 1: Time Travel with Rosbag

## The Flight Recorder

`ros2 bag` is a command-line tool that subscribes to topics and saves every message to a database (SQLite3 or MCAP). You can then "replay" this file, and your nodes won't know the difference between live data and recorded data.

## Recording Data

```bash
# Record specific topics
ros2 bag record /camera/image_raw /scan /odom

# Record ALL topics (Careful! Large files)
ros2 bag record -a
```

This creates a folder (e.g., `rosbag2_2025_12_09...`) containing a `.mcap` or `.db3` file and a `metadata.yaml`.

## Replaying Data

```bash
ros2 bag play rosbag2_2025_12_09-12_00_00
```

### Options

*   `--loop`: Play continuously.
*   `--rate 0.5`: Play at half speed (slow motion).
*   `--topics /scan`: Play only specific topics from the bag.

## Application: TDD with Bags

1.  Record a bag of the robot seeing a person.
2.  Write a "Person Detector" node.
3.  Play the bag.
4.  Check if your node detects the person.
5.  Refine the code.
6.  Repeat.

This allows you to develop perception algorithms at 3 AM without needing the physical robot (or the person).

## End-of-Lesson Checklist

- [ ] I have recorded a bag file containing at least two topics.
- [ ] I have verified the recording using `ros2 bag info <bag_folder>`.
- [ ] I have replayed the bag and verified (using `ros2 topic echo`) that data is being published.
- [ ] I understand the storage cost of recording high-bandwidth topics (like cameras).
