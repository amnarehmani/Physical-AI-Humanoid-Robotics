# Research: Module 2 (Digital Twin)

**Status**: Complete
**Date**: 2025-12-08

## 1. Technical Decisions

### Gazebo Version
- **Decision**: Use **Gazebo Fortress** (part of Gazebo Sim).
- **Rationale**: 
    - Replaces Gazebo Classic (EOL Jan 2025).
    - Officially supported by ROS 2 Humble.
    - Uses `ros_gz_bridge` which is the modern standard.
- **Alternatives**: Gazebo Classic (rejected due to EOL).

### Unity Integration
- **Decision**: Use **Unity Robotics Hub** packages (`ROS-TCP-Endpoint` and `ROS-TCP-Connector`).
- **Rationale**: 
    - Official solution from Unity Technologies.
    - Works over TCP/IP, allowing Unity to run on Windows while ROS 2 runs on WSL2/Linux.
    - Simple setup for beginners compared to native plugins.

### Sensor Simulation Strategy
- **Decision**: Use **Gazebo Plugins** mapped via `ros_gz_bridge`.
- **Sensors**:
    - **LiDAR**: `gpu_lidar` (faster) or `ray` sensor.
    - **Camera**: `camera` sensor.
    - **IMU**: `imu` sensor.
- **Workflow**: Define in URDF `<gazebo>` tags -> Spawn in Gazebo -> Bridge to ROS 2 topics -> Visualize in Rviz/Unity.

## 2. Key Concepts to Cover

### The Digital Twin Concept
- **Definition**: A virtual representation that matches the physical robot's physics and sensor properties.
- **Role**: Allows testing "dangerous" code without hardware risk.

### Simulation vs. Visualization
- **Gazebo**: The "Physics Engine" (Gravity, Collisions, Sensor generation).
- **Unity**: The "Renderer" (High-fidelity visuals, Human-Robot Interaction).
- **Rviz**: The "Debugger" (Raw sensor data visualization).

### Bridge Architecture
- **ROS 2 <-> Gazebo**: `ros_gz_bridge` (Topics).
- **ROS 2 <-> Unity**: `ROS-TCP-Endpoint` (JSON/BSON over TCP).

## 3. Artifacts & References
- **Gazebo Sim Docs**: https://gazebosim.org/docs
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
