---
id: m1-ch5-launch
title: "Lesson 3: Launch and Test"
sidebar_label: "Lesson 3: Launch & Test"
description: "Creating the final launch file and running the demo."
keywords:
  - launch
  - testing
  - simulation
  - turtlebot3
---

# Lesson 3: Launch and Test

## The Master Launch File

We need to start:
1.  Gazebo (Simulation).
2.  Robot State Publisher (URDF).
3.  Nav2 (Navigation Stack).
4.  Rviz2 (Visualization).
5.  **Our Patrol Node**.

```python
# patrol_app.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    return LaunchDescription([
        # Include standard Nav2 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/tb3_simulation_launch.py']),
            launch_arguments={'use_sim_time': 'True', 'headless': 'False'}.items(),
        ),
        
        # Start our Application
        Node(
            package='my_patrol_pkg',
            executable='patrol_node',
            name='patrol_brain',
            output='screen',
            parameters=[{'waypoints': [1.0, 0.0, 2.0, 2.0]}]
        )
    ])
```

## Running the Demo

1.  Build: `colcon build`
2.  Source: `source install/setup.bash`
3.  Launch: `ros2 launch my_patrol_pkg patrol_app.launch.py`

## Verification

*   **Rviz**: You should see the robot moving on the map.
*   **Terminal**: You should see logs like `[INFO] Reached Waypoint 1. Scanning...`
*   **Behavior**: The robot should not hit walls.

## Troubleshooting

*   **Robot spins forever**: The `ScannerNode` logic is broken. Check the timer.
*   **Robot sits still**: Nav2 might be waiting for an "Initial Pose". Use the "2D Pose Estimate" button in Rviz to tell the robot where it is.
*   **"Transform Error"**: Ensure `use_sim_time` is set to True for ALL nodes.

## End-of-Lesson Checklist

- [ ] I have created a Python launch file that includes other launch files.
- [ ] I can pass parameters (waypoints) from the launch file to the node.
- [ ] I have successfully completed a full patrol loop in simulation.
- [ ] I have taken a screenshot of the resulting path in Rviz.
