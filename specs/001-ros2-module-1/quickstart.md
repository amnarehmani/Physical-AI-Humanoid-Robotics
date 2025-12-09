# Quickstart: Module 1

**Purpose**: Verify the code examples and content environment.

## 1. Environment Setup
Ensure you have ROS 2 (Foxy or Humble) installed.
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

## 2. Running Python Examples
Navigate to the code directory:
```bash
cd code/module-1
```

**Publisher:**
```bash
python3 publisher.py
# Output: [INFO]: Publishing: "Move Forward"
```

**Subscriber:**
```bash
python3 subscriber.py
# Output: [INFO]: Received: "Move Forward"
```

## 3. Visualizing URDF
Install `urdf_tutorial` or use `rviz2`.
```bash
ros2 launch urdf_tutorial display.launch.py model:=simple_humanoid.urdf
```
*Outcome*: A window should open showing the blue torso and red head.
