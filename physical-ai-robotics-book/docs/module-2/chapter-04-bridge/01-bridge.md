---
id: m2-ch4-bridge
title: "Lesson 1: Configuring the Bridge"
sidebar_label: "Lesson 1: The Bridge"
description: "Mapping ROS topics to Gazebo topics."
keywords:
  - ros_gz_bridge
  - yaml
  - topics
  - mapping
---

# Lesson 1: Configuring the Bridge

## The Translation Problem

ROS 2 uses DDS (Data Distribution Service). Gazebo (Ignition) uses its own transport layer (Ignition Transport). They cannot talk directly.

The `ros_gz_bridge` node listens on both and translates.

## Running the Bridge via CLI

```bash
# syntax: ros2 run ros_gz_bridge parameter_bridge <topic_mapping>
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

This cryptic argument `/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist` means:
1.  **Topic**: `/cmd_vel`
2.  **ROS Type**: `geometry_msgs/msg/Twist`
3.  **Gazebo Type**: `gz.msgs.Twist`

Now, if you publish to `/cmd_vel` in ROS, it appears in Gazebo.

## YAML Configuration

Typing those arguments is error-prone. We use a YAML file for complex mappings.

```yaml
# bridge_config.yaml
- topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: BIDIRECTIONAL

- topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- topic_name: "/camera/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
```

## Launching with Config

```python
# bridge.launch.py
Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'config_file': 'path/to/bridge_config.yaml'}]
)
```

## Directionality

*   **ROS_TO_GZ**: Commands (Velocity, Joint positions).
*   **GZ_TO_ROS**: Sensors (Lidar, Camera, Odom).
*   **BIDIRECTIONAL**: Transforms (TF), Clock.

## End-of-Lesson Checklist

- [ ] I understand why a bridge is needed (different transport layers).
- [ ] I can write a bridge mapping argument on the command line.
- [ ] I can create a YAML file for multiple topic mappings.
- [ ] I know which direction data flows for sensors vs. actuators.
