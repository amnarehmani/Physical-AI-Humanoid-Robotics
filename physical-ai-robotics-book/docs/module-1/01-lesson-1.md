---
id: 01-lesson-1
title: "The Computing Graph"
sidebar_label: "1. Architecture"
description: "Understanding Nodes, Topics, and Services in ROS 2."
keywords:
  - ros2
  - nodes
  - topics
  - architecture
  - services
---

# Lesson 1: The Computing Graph

## 1.1 The "Nervous System" Metaphor

Imagine you are building a humanoid robot. You have:
*   A camera in the head.
*   Motors in the arms.
*   A battery in the back.
*   A computer in the chest.

How does the video from the camera get to the computer? How does the decision from the computer get to the motors? If you write one giant "main.py" script that does everything, it will be thousands of lines long, impossible to debug, and if one part crashes, the robot dies.

ROS 2 solves this by breaking the robot into **Nodes**.

## 1.2 Core Concepts

### Nodes (The Organs)
A **Node** is a small, focused program. Ideally, a node does *one thing*.
*   `camera_driver_node`: Reads data from hardware, publishes images.
*   `face_detector_node`: Subscribes to images, publishes "Face Found at X,Y".
*   `motor_controller_node`: Subscribes to "Target Angles", sends voltage to motors.

**Benefit**: If the `face_detector_node` crashes, the `motor_controller_node` keeps balancing the robot so it doesn't fall over.

### Topics (The Nerves)
How do nodes talk? They use **Topics**.
*   **Pattern**: Publish-Subscribe (Pub/Sub).
*   **Analogy**: A Radio Station.
    *   The **Publisher** broadcasts data on a frequency (Topic Name).
    *   The **Subscriber** tunes into that frequency.
    *   The Publisher *doesn't know or care* who is listening.
    *   This decouples the system. The camera doesn't need to know about the face detector.

<h3>Services (The Reflexes)</h3>
Sometimes you need a direct answer.
*   **Pattern**: Client-Server (Request-Response).
*   **Analogy**: A Phone Call. "Hello, what is the battery level?" -> "95%".
*   **Use Case**: Calibration, resetting simulations, changing modes.

<h2>1.3 The Computing Graph</h2>

When you run your system, all these nodes and topics form a network called the **Computing Graph**.

```mermaid
graph LR;
    CameraNode([Camera Node]) -->|/image_raw| FaceDetector([Face Detector]);
    FaceDetector -->|/cmd_face_pos| HeadTracker([Head Tracker]);
    HeadTracker -->|/motor_cmd| NeckMotors([Neck Motors]);
```

<h2>1.4 Hands-On: Running Your First Node</h2>

Let's verify your ROS 2 installation by running a Python node. We will write a simple node that announces its existence.

<h3>Step 1: Create the File</h3>
Create a file named `simple_node.py`.

```python title="code/module-1/simple_node.py"
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        # Initialize the node with the name 'simple_node'
        super().__init__('simple_node')
        # Log a message to the console
        self.get_logger().info('The Nervous System is Active!')

def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of our node
    node = SimpleNode()
    
    # Keep the node running (listening for events)
    # Since this node does nothing else, it will just sit here
    # untill you press Ctrl+C
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h3>Step 2: Execution</h3>
Run the script using Python 3. Ensure you have sourced your ROS 2 environment (e.g., `source /opt/ros/humble/setup.bash`).

```bash
python3 simple_node.py
```

<h3>Step 3: Verification</h3>
**Expected Output:**
```text
[INFO] [163...]: The Nervous System is Active!
```

You have successfully launched a node into the ROS 2 graph. In the next lesson, we will make it talk.
