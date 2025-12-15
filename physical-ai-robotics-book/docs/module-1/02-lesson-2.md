---
id: 02-lesson-2
title: "Controlling the Body"
sidebar_label: "2. Python Control"
description: "Using rclpy to implement Publishers and Subscribers in ROS 2."
keywords:
  - ros2
  - python
  - rclpy
  - publisher
  - subscriber
  - control
---

# Lesson 2: Controlling the Body (Python)

## 1. Introduction

In the previous lesson, we built a single node—a lonely neuron firing into the void. Today, we build the synapse.

The ability to transmit intent from one process to another is the definition of a distributed system. In humanoid robotics, this is not just about sending text; it is about transmitting the will to move from a high-level planner (The Brain) to a low-level motor controller (The Muscle). This lesson bridges the gap between thought and action using the **Publisher-Subscriber** pattern.

## 2. Conceptual Understanding: The Radio Station

**Intuition**:
Imagine a radio station broadcasting on 101.5 FM.
*   **The Station (Publisher)**: It blasts music into the air. It does not know if you are listening. It does not know if you are in a tunnel. It just transmits.
*   **The Car (Subscriber)**: It tunes into 101.5 FM. It receives the music. It cannot talk back to the station.

**The "Topic"**:
In ROS 2, the frequency (101.5 FM) is the **Topic**.
*   If the Station broadcasts on "101.5" and the Car listens to "101.6", there is silence.
*   If the Station broadcasts "Jazz" (Message Type A) and the Car expects "News" (Message Type B), there is confusion (or a crash).

## 3. System Perspective: The Control Loop Architecture

Before writing code, let's visualize the data pipeline we are building.

```mermaid-text
+-----------------------+                    +-----------------------+
|   Node: Brain (Pub)   |                    |  Node: Muscle (Sub)   |
+-----------------------+                    +-----------------------+
|                       |                    |                       |
|  [Timer: 1.0 sec]     |                    |   [Wait for Event]    |
|         |             |                    |           |           |
|         v             |                    |           v           |
|   create_message()    |                    |    execute_command()  |
|         |             |                    |           ^           |
|         v             |      Topic         |           |           |
|    publish(msg)       |------------------->|    (Callback)         |
|                       |  "motor_commands"  |                       |
+-----------------------+                    +-----------------------+
```

### The Data Flow
1.  **Trigger**: The Brain node has a timer tick (every 1s).
2.  **Generation**: The Brain creates a string payload "Move Forward".
3.  **Serialization**: `rclpy` converts Python string -> DDS Bytes.
4.  **Transport**: DDS sends bytes over the network stack (localhost).
5.  **Deserialization**: `rclpy` converts DDS Bytes -> Python string.
6.  **Callback**: The Muscle node interrupts its sleep to run `execute_command()`.

## 4. Practical Example: Implementing the Nervous System

We will implement two separate Python scripts. They will run as independent processes.

### 4.1 The Brain (Publisher)
This node represents the cognitive layer. It decides *what* to do.

```python title="code/module-1/publisher.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        # 1. Create a publisher
        # Type: String (The message format)
        # Topic Name: 'motor_commands' (The frequency)
        # Queue Size: 10 (Buffer size if network is busy)
        self.publisher_ = self.create_publisher(String, 'motor_commands', 10)
        
        # 2. Create a timer to fire every 1.0 seconds
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Step Forward {self.i}'
        
        # 3. Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Brain Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    brain_node = BrainNode()
    try:
        rclpy.spin(brain_node)
    except KeyboardInterrupt:
        pass
    finally:
        brain_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 The Muscle (Subscriber)
This node represents the actuation layer. It executes orders.

```python title="code/module-1/subscriber.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MuscleNode(Node):
    def __init__(self):
        super().__init__('muscle_node')
        
        # 1. Create subscription
        self.subscription = self.create_subscription(
            String,             # Must match Publisher type
            'motor_commands',   # Must match Publisher topic
            self.listener_callback,
            10)                 # Queue size
        
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # 2. React to the message
        # This function runs AUTOMATICALLY when data arrives
        self.get_logger().info(f'Muscle Received: "{msg.data}" -> FLEXING LEG!')

def main(args=None):
    rclpy.init(args=args)
    muscle_node = MuscleNode()
    try:
        rclpy.spin(muscle_node)
    except KeyboardInterrupt:
        pass
    finally:
        muscle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Engineering Insights: Latency and Safety

In a toy example, we send strings every second. In a real humanoid, we send `JointTrajectory` messages at 500Hz.

### 5.1 The Latency Budget
If the "Brain" sees an obstacle and publishes "STOP", that message travels through:
1.  Brain Node Serialization overhead.
2.  OS Network Stack latency.
3.  Muscle Node Deserialization overhead.
4.  Muscle Node Callback execution time.

If this total time > 100ms, the robot might hit the obstacle before the brakes engage. This is why we keep callbacks **short** and **non-blocking**.

### 5.2 Threading and Blocking
**CRITICAL RULE**: Never use `time.sleep()` or `while(True)` inside a callback.
*   If `listener_callback` sleeps for 5 seconds, the node stops "hearing" new messages for 5 seconds.
*   The queue fills up (up to 10 messages).
*   The 11th message (which might be "Emergency Stop") is dropped.

## 6. Summary

You have built the communication backbone of your robot.
1.  **Publishers** broadcast intent.
2.  **Subscribers** react to intent.
3.  **Callbacks** are the event-driven functions that handle data.

We have a brain and a muscle. Now we need a skeleton. In the next lesson, we will define the robot's physical structure using URDF.
