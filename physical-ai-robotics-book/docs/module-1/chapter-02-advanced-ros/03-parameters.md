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
  - configuration
---

# Lesson 3: Parameters and Launch Files

## 1. Introduction

You have built nodes that talk (Topics), nodes that answer (Services), and nodes that work (Actions). But as your system grows from 2 nodes to 50, a new problem emerges: **Configuration**.

Imagine you write a camera driver. You hard-code the resolution to `1920x1080`.
*   **Day 1**: It works great on your desktop.
*   **Day 2**: You move to a Raspberry Pi. It crashes because it only supports `640x480`. You edit the code.
*   **Day 3**: You add a second camera. You need `camera_1` on topic `/front/image` and `camera_2` on `/back/image`. You copy-paste the code and edit the strings.

This is "Spaghetti Robotics."

In this lesson, we will separate **Logic** (Code) from **Configuration** (Parameters). We will then use **Launch Files** to orchestrate dozens of nodes with a single command.

## 2. Conceptual Understanding: The Tuning Knobs

**Intuition**:
Think of your node as a guitar amplifier.
*   **The Code**: The circuitry inside. It amplifies signals. You don't solder new resistors every time you want to change the volume.
*   **The Parameters**: The knobs on the front. Volume, Gain, Bass, Treble. You tweak these at runtime to suit the room.

**Mechanism**:
A ROS 2 Parameter is a key-value pair hosted inside a node.
*   It has a **Type** (Int, Float, String, Boolean, Array).
*   It has a **Name** (`max_speed`, `camera_frame_id`).
*   It can be **Read**, **Set**, and **Listed** externally via tools or other nodes.

## 3. System Perspective: The Launch Hierarchy

How do we manage 50 nodes? We use a hierarchy of Launch Files.

```mermaid-text
[Main Robot Launch] (bringup.launch.py)
       |
       +---> [Sensors Launch] (sensors.launch.py)
       |          |
       |          +---> [Lidar Node] (params: ip=192.168.1.10)
       |          +---> [Camera Node] (params: res=1080p)
       |
       +---> [Navigation Launch] (nav.launch.py)
                  |
                  +---> [Planner Node] (params: max_speed=2.0)
                  +---> [Controller Node] (params: p_gain=0.5)
```

This structure allows you to "Include" other launch files, passing down configuration like a waterfall.

## 4. Practical Example: The Parameterized Node

Let's build a node that behaves differently based on its settings.

### 4.1 The Python Code

```python title="code/module-1/parameterized_node.py"
import rclpy
from rclpy.node import Node

class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')
        
        # 1. Declare Parameters (with default values)
        # Without declaration, ROS 2 will reject external attempts to set them
        self.declare_parameter('robot_name', 'generic_bot')
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('waypoints', [0, 1, 2, 3])

        # 2. Get Initial Values
        name = self.get_parameter('robot_name').value
        speed = self.get_parameter('max_speed').value
        
        self.get_logger().info(f'Hello! I am {name}. I can run at {speed} m/s.')

        # 3. Parameter Callback (Optional)
        # Allow dynamic reconfigure at runtime
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            if param.name == 'max_speed':
                if param.value < 0.0:
                    self.get_logger().warn('Speed cannot be negative!')
                    return rclpy.parameter.SetParametersResult(successful=False)
                self.get_logger().info(f'Speed updated to: {param.value}')
        return rclpy.parameter.SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 4.2 The Launch File

Now, let's launch two instances of this same node with different personalities.

```python title="code/module-1/dual_robot.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robot 1: Fast Runner
        Node(
            package='my_robot_package',
            executable='configurable_node',
            name='runner_bot',
            parameters=[
                {'robot_name': 'Usain_Bolt'},
                {'max_speed': 10.0}
            ]
        ),
        # Robot 2: Slow Walker
        Node(
            package='my_robot_package',
            executable='configurable_node',
            name='walker_bot',
            parameters=[
                {'robot_name': 'Turtle'},
                {'max_speed': 0.1}
            ]
        )
    ])
```

## 5. Engineering Insights: Configuration Management

*   **YAML Files**: For complex nodes (like Nav2), putting parameters in Python launch files is messy. Use YAML files instead:
    ```yaml
    /runner_bot:
      ros__parameters:
        max_speed: 10.0
        pid_gains: [1.0, 0.0, 0.5]
    ```
*   **Namespacing**: When running multiple robots, use namespaces (`/robot1/camera`, `/robot2/camera`) in your launch files to prevent topic collisions.
*   **Dynamic Reconfigure**: Being able to tune PID gains or vision thresholds *while the robot is running* is a superpower during field testing. Use `ros2 param set` CLI or the `rqt_reconfigure` GUI tool.

## 6. Summary

You have moved from writing scripts to building systems.
1.  **Parameters** separate code from configuration.
2.  **Launch Files** separate orchestration from execution.
3.  **YAML** allows us to save and share configurations.

This concludes the Advanced ROS 2 Concepts chapter. You now possess the "Communication Trinity" (Topics, Services, Actions) and the "Management Tools" (Parameters, Launch). You are ready to orchestrate a digital twin.