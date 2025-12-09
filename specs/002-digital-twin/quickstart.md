# Quickstart: Module 2

**Purpose**: Verify the Digital Twin setup.

## 1. Launching Simulation (Gazebo)
```bash
# Terminal 1
ros2 launch code/module-2/simulation.launch.py
```
*Expectation*: Gazebo Fortress opens. Robot spawns on ground plane. Gravity applies (robot settles).

## 2. Verifying Sensors
```bash
# Terminal 2
ros2 topic list
# Expect: /scan, /camera/image_raw, /imu/data

ros2 topic hz /scan
# Expect: ~10Hz
```

## 3. Launching Visualization (Unity)
1. Open Unity Hub -> Open `code/module-2/unity_project`.
2. Press "Play" in Editor.
3. In Terminal 2, publish a fake movement:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```
*Expectation*: Unity robot moves slightly (if controller script is active) or at least connects without error console logs.
