# Code Artifacts Contract

This module provides the following public code examples.

## 1. Python Nodes

### `simple_node.py`
- **Purpose**: Minimal ROS 2 Node.
- **Dependencies**: `rclpy`
- **Input**: None
- **Output**: Log message "Node started".

### `publisher.py`
- **Purpose**: Publishes data to a topic.
- **Topic**: `/motor_commands` (std_msgs/String)
- **Rate**: 2Hz
- **Logic**: Publishes dummy command string.

### `subscriber.py`
- **Purpose**: Listens for data.
- **Topic**: `/motor_commands` (std_msgs/String)
- **Logic**: Logs received message.

## 2. URDF Models

### `simple_humanoid.urdf`
- **Purpose**: Visualization of robot structure.
- **Links**: `torso`, `head`, `right_upper_arm`.
- **Joints**: `neck_joint` (revolute), `right_shoulder_joint` (revolute).
