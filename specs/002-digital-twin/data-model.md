# Data Model: Module 2 Content Structure

## 1. Hierarchy

### Module: The Digital Twin
- **ID**: `module-2`
- **Slug**: `/modules/digital-twin`
- **Metadata**:
    - `title`: "The Digital Twin (Gazebo & Unity)"
    - `difficulty`: "Intermediate"
    - `prerequisites`: ["Module 1 (ROS 2 Basics)"]

## 2. Lesson Definitions

### Lesson 1: Physics & Environment
- **Filename**: `lesson-1-physics.md`
- **Title**: "Building the Physics Playground (Gazebo)"
- **Key Concepts**: Physics Engines, SDF vs URDF, World Files.
- **Artifacts**: `simulation.launch.py`, `basic_world.sdf`.

### Lesson 2: Perception
- **Filename**: `lesson-2-sensors.md`
- **Title**: "Simulating Perception (Sensors)"
- **Key Concepts**: Lidar, Depth Camera, IMU, Plugins.
- **Artifacts**: `sensors.xacro` (Gazebo tags).

### Lesson 3: Visualization
- **Filename**: `lesson-3-unity.md`
- **Title**: "The Digital Mirror (Unity)"
- **Key Concepts**: Digital Twin, ROS-TCP, Asset Import.
- **Artifacts**: `UnitySetupScript.cs` (or instructions).

## 3. Entity Relationships
- **URDF (Module 1)** --[extended by]--> **Sensors.xacro (Module 2)**
- **ROS 2 Topics** --[bridged to]--> **Gazebo**
- **ROS 2 Topics** --[bridged to]--> **Unity**
