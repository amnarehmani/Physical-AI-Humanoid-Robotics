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
---

# Lesson 2: Controlling the Body (Python)

<h2>2.1 Introduction to `rclpy`</h2>

`rclpy` (ROS Client Library for Python) is the tool we use to interface Python scripts with the ROS 2 core. It handles the complex networking (DDS - Data Distribution Service) so we can focus on logic.

In this lesson, we will build a "Brain" (Publisher) and a "Muscle" (Subscriber).

<h2>2.2 The Publisher (The Brain)</h2>

The Brain decides what to do. It sends commands out to the system.

<h3>Code Breakdown</h3>
1.  **Import Message Type**: We need a standard format. We'll use `std_msgs.msg.String` for simple text.
2.  **Create Publisher**: `self.create_publisher(Type, TopicName, QueueSize)`.
3.  **Timer**: We don't want a `while True` loop blocking everything. We use a timer callback to publish at a fixed frequency (e.g., 1 Hz).

```python title="code/module-1/publisher.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        # Create a publisher that sends Strings to the 'motor_commands' topic
        # The '10' is the queue size (buffer)
        self.publisher_ = self.create_publisher(String, 'motor_commands', 10)
        
        # Create a timer that fires every 1.0 seconds
        self.timer = self.create_timer(1.0, self.send_command)
        self.counter = 0

    def send_command(self):
        msg = String()
        msg.data = f'Move Forward step {self.counter}'
        
        # Publish the message
        self.publisher_.publish(msg)
        
        self.get_logger().info(f'Brain sending: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h2>2.3 The Subscriber (The Muscle)</h2>

The Muscle doesn't think; it waits for orders.

<h3>Code Breakdown</h3>
1.  **Create Subscription**: `self.create_subscription(Type, TopicName, Callback, QueueSize)`.
2.  **Callback**: This function `execute_command(self, msg)` is triggered *automatically* whenever a new message arrives.

```python title="code/module-1/subscriber.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MuscleNode(Node):
    def __init__(self):
        super().__init__('muscle_node')
        
        # Listen to the 'motor_commands' topic
        self.subscription = self.create_subscription(
            String,
            'motor_commands',
            self.execute_command,
            10)
        # Prevent unused variable warning
        self.subscription 

    def execute_command(self, msg):
        # This function runs every time the Brain sends a message
        self.get_logger().info(f'Muscle received: "{msg.data}" -> FLEXING MUSCLES!')

def main(args=None):
    rclpy.init(args=args)
    node = MuscleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h2>2.4 Hands-On: Connecting the Nervous System</h2>

To see this work, we need **two terminal windows**.

1.  **Terminal 1 (The Muscle)**:
    ```bash
    python3 subscriber.py
    ```
    *It will sit waiting. Silence.*

2.  **Terminal 2 (The Brain)**:
    ```bash
    python3 publisher.py
    ```

**Result**:
*   Terminal 2 prints: `[INFO]: Brain sending: "Move Forward step 0"`
*   Terminal 1 *immediately* prints: `[INFO]: Muscle received: "Move Forward step 0" -> FLEXING MUSCLES!`

You have just created a distributed computing system. These two scripts could theoretically be on different continents connected via VPN, and the logic would be identical.
