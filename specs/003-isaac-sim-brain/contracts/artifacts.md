# Code Artifacts Contract: Module 3

## 1. Isaac ROS Artifacts

### `visual_slam.launch.py`
- **Purpose**: Starts the `isaac_ros_visual_slam` node.
- **Inputs**: 
    - Left Image: `/camera/left/image_rect`
    - Right Image: `/camera/right/image_rect`
    - IMU: `/camera/imu`
- **Outputs**: 
    - Odometry: `/visual_slam/odom`
    - TF: `odom` -> `base_link`

## 2. Nav2 Artifacts

### `nav2_humanoid_params.yaml`
- **Purpose**: Configuration for Navigation 2 stack tailored for a humanoid.
- **Key Settings**:
    - `robot_radius`: Replaced by `footprint` polygon.
    - `inflation_radius`: 0.55m (Safety margin).
    - `controller_plugin`: `DWB` (with conservative velocity limits).

## 3. Isaac Sim Artifacts

### `load_humanoid.py` (Python Script)
- **Purpose**: Loads a USD humanoid asset into the stage via code.
- **Library**: `omni.isaac.core`.
