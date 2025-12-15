# Specification: Isaac Sim Brain Module (Module 3)

**Feature**: Isaac Sim Brain Module
**Status**: DRAFT
**Module**: 003-isaac-sim-brain
**Priority**: P1

## 1. Overview

This module focuses on the "Brain" of the humanoid robot, bridging the gap between high-level simulation (NVIDIA Isaac Sim) and the robot's nervous system (ROS 2). The goal is to teach students how to create a "Digital Twin" brain that can perceive the simulated world and make decisions.

### 1.1 Context
- **Previous Module**: Module 2 (Digital Twin) established the physical simulation environment.
- **Next Module**: Module 4 (VLA) will introduce advanced AI models.
- **This Module**: Connects the simulation to the ROS 2 graph, enabling sensor data flow (Cameras, Lidar) and control command execution.

## 2. Learning Objectives

By the end of this module, the student will be able to:
1.  **Understand** the Omniverse architecture and how Isaac Sim interacts with ROS 2.
2.  **Implement** a ROS 2 Bridge to stream high-fidelity sensor data (RGB-D, Lidar) from Isaac Sim to ROS 2 topics.
3.  **Create** a synthetic data generation pipeline using Isaac Replicator for training AI models.
4.  **Develop** a basic navigation or manipulation task within the simulator controlled by external ROS 2 nodes.

## 3. User Stories

### US1: The Bridge
**As a** robotics engineer,
**I want** to stream camera images from Isaac Sim to Rviz2,
**So that** I can verify my perception pipeline using synthetic data.
- *Acceptance Criteria*: A ROS 2 node receives `/camera/rgb` and `/camera/depth` topics from a running Isaac Sim instance.

### US2: Synthetic Eyes (Replicator)
**As a** ML researcher,
**I want** to generate a dataset of labeled images (bounding boxes),
**So that** I can train a vision model before building the physical robot.
- *Acceptance Criteria*: A script generates 100 annotated images of a target object in randomized lighting conditions.

### US3: The First Thought (Navigation)
**As a** system integrator,
**I want** to send velocity commands from a ROS 2 terminal to move the simulated robot,
**So that** I can test my control logic without risking hardware damage.
- *Acceptance Criteria*: Publishing to `/cmd_vel` moves the robot in Isaac Sim.

## 4. Technical Constraints

- **Platform**: NVIDIA Isaac Sim (latest stable), ROS 2 Humble.
- **Hardware**: Requires NVIDIA RTX GPU (for Isaac Sim).
- **Format**: Lessons must follow the "Immutable Structure" (Intuition -> Theory -> System -> Example -> Limitations).
- **Code**: Python-based `omni` kit scripting and ROS 2 `rclpy` nodes.

## 5. File Structure (Target)

```text
docs/module-3/
├── 00-intro.md
├── 01-lesson-1.md (The Bridge)
├── 02-lesson-2.md (Replicator)
├── 03-lesson-3.md (Navigation/Control)
└── 04-summary.md

code/module-3/
├── isaac_ros/
│   └── bridge_config.yaml
├── replicator/
│   └── generate_data.py
└── nav2/
    └── simple_nav.py
```
