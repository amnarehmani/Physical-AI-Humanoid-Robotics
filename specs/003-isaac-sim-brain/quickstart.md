# Quickstart: Module 3

**Purpose**: Verify the Isaac AI pipeline.

## 1. Launching Isaac Sim
1. Open Omniverse Launcher.
2. Launch **Isaac Sim**.
3. Open `Window -> Extensions`, enable `omni.isaac.ros2_bridge`.

## 2. Running Visual SLAM
Ensure you are in the Isaac ROS Docker container.
```bash
ros2 launch code/module-3/visual_slam.launch.py
```
*Verification*: Open Rviz2. You should see the `odom` frame moving as the robot moves in Isaac Sim.

## 3. Running Nav2
```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=code/module-3/nav2_humanoid_params.yaml
```
*Verification*: Set a "2D Nav Goal" in Rviz2. The robot in Isaac Sim should move towards it.
