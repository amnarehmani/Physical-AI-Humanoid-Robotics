---
id: m1-ch2-parameters
title: "Lesson 3: Parameters and Launch Files"
sidebar_label: "Lesson 3: Parameters"
description: "Dynamic configuration and system orchestration."
keywords:
  - ros2
  - parameters
  - launch files
  - python
  - yaml
---

# Lesson 3: Parameters and Launch Files

## The Hard-Coding Problem

Imagine you wrote a camera driver node. You hard-coded the resolution to `1920x1080`.
Now you want to run it on a Raspberry Pi that only supports `640x480`. You have to edit the code.
Then you want to change the topic name from `/camera/image_raw` to `/front_cam/image`. You edit the code again.

This is bad practice. In Robotics, we separate **Logic** (Code) from **Configuration** (Parameters).

## ROS 2 Parameters

A Parameter is a configuration value stored inside a node. Parameters can be integers, floats, booleans, strings, or arrays.

### Using Parameters in Python

```python
class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('my_param_node')
        
        # 1. Declare the parameter with a default value
        self.declare_parameter('my_speed', 1.0)
        self.declare_parameter('robot_name', 'robot_1')

        # 2. Access the parameter
        speed = self.get_parameter('my_speed').value
        name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.get_logger().info(f'Robot {name} speed is {speed}')

        # 3. Create a timer to check for updates (optional)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # You can fetch the parameter again to see if it changed
        new_speed = self.get_parameter('my_speed').value
        # ... logic ...
```

### Changing Parameters via CLI

You can change parameters at runtime without restarting the node:

```bash
# List params
ros2 param list

# Get param
ros2 param get /my_param_node my_speed

# Set param
ros2 param set /my_param_node my_speed 5.0
```

## Launch Files: The Orchestrator

A complex robot might have 50 nodes (Camera, Lidar, Motor Controller, Planner, SLAM, etc.).
Opening 50 terminal tabs is impossible.
**Launch Files** allow you to start multiple nodes with a single command and configure their parameters simultaneously.

### Python Launch Files

In ROS 2, launch files are Python scripts. This gives them immense power (logic, loops, conditionals).

Example `robot_app.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node 1: The Service Server
        Node(
            package='my_robot_package',
            executable='simple_service_server',
            name='server_node',
            output='screen'
        ),
        # Node 2: The Parameterized Node
        Node(
            package='my_robot_package',
            executable='my_param_node',
            name='config_node',
            parameters=[
                {'my_speed': 2.5},
                {'robot_name': 'R2D2'}
            ]
        )
    ])
```

### Running a Launch File

```bash
ros2 launch my_robot_package robot_app.launch.py
```

This single command spins up the entire system.

## YAML Configuration

For large systems, even Python launch files get messy. We can store parameters in a `.yaml` file:

```yaml
# config/robot_params.yaml
/config_node:
  ros__parameters:
    my_speed: 10.0
    robot_name: "C3PO"
```

And load it in the launch file:
```python
parameters=[os.path.join(get_package_share_directory('pkg'), 'config', 'robot_params.yaml')]
```

## End-of-Lesson Checklist

- [ ] I can modify a node to use `declare_parameter` instead of hard-coded variables.
- [ ] I have used the CLI `ros2 param set` to change a running node's behavior.
- [ ] I can write a simple Python Launch file to start two nodes at once.
- [ ] I understand the role of YAML files in storing robot configurations.

This concludes Chapter 2. You now have the architectural tools (Services, Actions, Parameters, Launch) to build robust systems.
