# Code Artifacts Contract: Module 2

## 1. ROS 2 / Gazebo Artifacts

### `simulation.launch.py`
- **Purpose**: Spawns the robot in Gazebo and starts the bridge.
- **Inputs**: URDF file path.
- **Outputs**: Gazebo process, `ros_gz_bridge` process.

### `sensors.xacro`
- **Purpose**: Adds sensor plugins to the base robot.
- **Sensors**:
    - **Lidar**: `/scan` (LaserScan)
    - **Camera**: `/camera/image_raw` (Image)
    - **IMU**: `/imu/data` (Imu)

## 2. Unity Artifacts

### `UnityROSConnection` (Configuration)
- **Purpose**: Configures ROS-TCP-Connector.
- **Settings**:
    - IP: `127.0.0.1` (or host IP)
    - Port: `10000`

### `RobotController.cs` (Script)
- **Purpose**: Subscribes to joint states/pose to move the Unity ArticulationBody.
- **Input**: `/joint_states` or TF.
