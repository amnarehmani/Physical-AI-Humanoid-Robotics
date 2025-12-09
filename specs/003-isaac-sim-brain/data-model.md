# Data Model: Module 3 Content Structure

## 1. Hierarchy

### Module: The AI-Robot Brain
- **ID**: `module-3`
- **Slug**: `/modules/isaac-brain`
- **Metadata**:
    - `title`: "The AI-Robot Brain (NVIDIA Isaac)"
    - `difficulty`: "Advanced"
    - `prerequisites`: ["Module 2 (Digital Twin)", "NVIDIA RTX GPU"]

## 2. Lesson Definitions

### Lesson 1: Photorealistic Simulation
- **Filename**: `lesson-1-isaac-sim.md`
- **Title**: "Building Realities (Isaac Sim)"
- **Key Concepts**: USD, Nucleus, RTX Rendering, Synthetic Data.
- **Artifacts**: `warehouse_scene.usd`.

### Lesson 2: Visual Intelligence
- **Filename**: `lesson-2-isaac-ros.md`
- **Title**: "Accelerated Perception (Isaac ROS)"
- **Key Concepts**: VSLAM, Hardware Acceleration, GEMs.
- **Artifacts**: `visual_slam.launch.py`.

### Lesson 3: Autonomous Navigation
- **Filename**: `lesson-3-nav2.md`
- **Title**: "Planning the Path (Nav2)"
- **Key Concepts**: Costmaps, Behavior Trees, Planners vs Controllers.
- **Artifacts**: `nav2_humanoid_params.yaml`.

## 3. Entity Relationships
- **Isaac Sim** --[publishes Stereo Images]--> **Isaac ROS VSLAM**
- **Isaac ROS VSLAM** --[publishes /tf, /odom]--> **Nav2**
- **Nav2** --[publishes /cmd_vel]--> **Isaac Sim (Robot Controller)**
