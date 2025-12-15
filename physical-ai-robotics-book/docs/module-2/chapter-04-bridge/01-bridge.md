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

## 1. Introduction

The `ros_gz_bridge` is the diplomat of our simulation. Without it, ROS 2 commands scream into the void, and Gazebo sensors display data to no one.

In this lesson, we will master the configuration of this critical node. We will learn how to map topic names, convert message types, and manage data directionality.

## 2. Conceptual Understanding: The Mapping Syntax

The bridge needs to know three things to translate a message:
1.  **Topic Name**: What is the channel called? (e.g., `/cmd_vel`)
2.  **ROS Type**: What is the data structure in ROS? (e.g., `geometry_msgs/msg/Twist`)
3.  **Gazebo Type**: What is the data structure in Gazebo? (e.g., `gz.msgs.Twist`)

### The Syntax
The bridge accepts arguments in a specific format:
`TOPIC@ROS_TYPE[GAZEBO_TYPE`

*   `@`: Bidirectional.
*   `[`: Gazebo to ROS (Sensors).
*   `]`: ROS to Gazebo (Commands).

## 3. System Perspective: The YAML Config

For a real robot, the command line gets too long. We use a YAML configuration file.

```yaml
# bridge_config.yaml
- topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ  # Commands go TO the sim

- topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS  # Sensor data comes FROM the sim

- topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS  # Sim time drives ROS time
```

## 4. Implementation: The Launch File

We rarely run `ros2 run` manually. We wrap it in a Python launch file.

```python
# simulation.launch.py excerpt
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{
        'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'),
        'qos_overrides./scan.reliability': 'best_effort'
    }],
    output='screen'
)
```

**Note on QoS**: Quality of Service (QoS) is critical. Gazebo often publishes sensor data as "Best Effort" (UDP-like). If ROS expects "Reliable" (TCP-like), the connection fails. The bridge allows you to override QoS settings.

## 5. Real-World Example: Debugging a Dead Bridge

**Scenario**: You launch your simulation. Rviz is black. No errors.
**Diagnosis**:
1.  Check ROS: `ros2 topic echo /scan` -> No data.
2.  Check Gazebo: `gz topic -e -t /scan` -> Data is streaming!
3.  **Conclusion**: The bridge is broken.
4.  **Fix**: Check the `ros_gz_bridge` output log. Did it say "Failed to create subscriber"? Usually, this means a typo in the message type (e.g., `LaserScan` vs `Laserscan`).

## 6. Engineering Insights: Performance

The bridge does **serialization**. It takes a binary blob from Gazebo, unpacks it, repacks it into a ROS binary blob, and sends it out.
*   **Small Messages (IMU, Odom)**: Negligible cost.
*   **Large Messages (Images, PointClouds)**: High cost.

**Optimization**: Use `image_bridge` (a specialized executable) instead of `parameter_bridge` for cameras. It uses shared memory where possible to optimize image transfer.

## 7. Summary

The Bridge is the lifeline.
*   **Commands** flow Down (ROS -> GZ).
*   **Sensors** flow Up (GZ -> ROS).
*   **Time** flows Up (GZ -> ROS).

In the next lesson, we will learn how to programmatically inject robots into the world using the `create` node.