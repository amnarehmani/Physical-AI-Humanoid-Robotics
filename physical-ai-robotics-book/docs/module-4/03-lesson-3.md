---
id: lesson-3
title: "Capstone: The Butler Robot"
sidebar_label: "3. Capstone Project"
description: "Full integration of Voice, Planning, and Action for an autonomous robot."
keywords:
  - capstone
  - integration
  - project
  - ros2
  - vla
---

# Lesson 3: Capstone: The Butler Robot

## Introduction

We have built the **Ear** (Lesson 1) and the **Brain** (Lesson 2). Now we need the **Body**.
In this capstone lesson, we will integrate all components into a single functioning system: **The Executive Node**.

The Executive is the "conductor" of the orchestra. It receives the score (the JSON plan) from the Brain and signals the musicians (Nav2, Arm Controllers) when to play.

## System Perspective: The VLA Architecture

Here is the complete data flow for our VLA agent:

```text
       "Get me a soda"
             |
      [ Whisper Node ]
             | /speech/text
             v
      [ Planner Node ]
             | /planning/plan (JSON)
             v
+--------------------------+
|    EXECUTIVE NODE        |
|  1. Parse JSON List      |
|  2. Loop through Actions |
|  3. Call Hardware APIs   |
+--------------------------+
       |            |
       v            v
  [ Nav2 Stack ]  [ Arm Ctrl ]
```

## Implementation: The Executive Node

For this lesson, we will "mock" the physical actions (Navigation and Grasping) with timers. This allows you to test the VLA logic without needing a heavy simulator running in the background. In the "Extensions" section, we explain how to swap these mocks for real ROS 2 Action Clients.

```python title="code/module-4/execution/executive_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class ExecutiveNode(Node):
    def __init__(self):
        super().__init__('executive_node')
        
        # Subscribe to the plan
        self.subscription = self.create_subscription(
            String, '/planning/plan', self.execute_callback, 10)
            
        # Hardcoded map for the mock navigation
        self.location_coords = {
            "kitchen": (5.0, 2.0),
            "bedroom": (-2.0, 3.0),
            "table": (1.5, 1.5),
            "user": (0.0, 0.0) # Assume user is at origin
        }
        
        self.get_logger().info("Executive is ready to serve.")

    def execute_callback(self, msg):
        try:
            plan = json.loads(msg.data)
            self.get_logger().info(f"Received plan with {len(plan)} steps.")
            
            # Execute steps sequentially
            for i, step in enumerate(plan):
                self.get_logger().info(f"--- Step {i+1}/{len(plan)} ---")
                self.execute_step(step)
                
            self.get_logger().info("Mission Complete!")
            
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received.")

    def execute_step(self, step):
        action = step.get("action")
        
        if action == "navigate":
            target_name = step.get("target")
            coords = self.location_coords.get(target_name, (0,0))
            self.mock_navigation(target_name, coords)
            
        elif action == "pick":
            object_name = step.get("object")
            self.mock_manipulation(f"Picking up {object_name}")
            
        elif action == "place":
            object_name = step.get("object")
            self.mock_manipulation(f"Placing {object_name}")
            
        else:
            self.get_logger().warn(f"Unknown action: {action}")

    def mock_navigation(self, target, coords):
        self.get_logger().info(f"NAVIGATING to '{target}' at {coords}...")
        # In a real robot, this would be: client.send_goal(coords)
        time.sleep(2.0) # Simulate travel time
        self.get_logger().info(f"Arrived at {target}.")

    def mock_manipulation(self, task):
        self.get_logger().info(f"ARM ACTION: {task}...")
        # In a real robot, this would be: moveit_group.execute(traj)
        time.sleep(1.0) # Simulate grasping time
        self.get_logger().info("Grasp complete.")

def main():
    rclpy.init()
    node = ExecutiveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Real-World Application: The "Butler" Workflow

1.  **Launch Terminal 1 (Ear)**:
    `python3 code/module-4/voice/whisper_node.py`
    *(Make sure OPENAI_API_KEY is exported)*

2.  **Launch Terminal 2 (Brain)**:
    `python3 code/module-4/planning/llm_planner.py`

3.  **Launch Terminal 3 (Body)**:
    `python3 code/module-4/execution/executive_node.py`

4.  **Action**:
    Speak clearly into your microphone: *"Go to the kitchen and pick up the soda."*

5.  **Observation**:
    *   **T1**: `Heard: "Go to the kitchen and pick up the soda."`
    *   **T2**: `Generated Plan: [{"action": "navigate", "target": "kitchen"}, {"action": "pick", "object": "soda"}]`
    *   **T3**: `NAVIGATING to 'kitchen'... (2s wait) ... Arrived. ARM ACTION: Picking up soda...`

## Engineering Extensions

To make this "real," you simply replace the `mock_navigation` function with a ROS 2 Action Client for Nav2.

```python
# Real Navigation Example (Pseudo-code)
from nav2_msgs.action import NavigateToPose

def navigate_real(self, x, y):
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    
    self.nav_client.wait_for_server()
    self.nav_client.send_goal_async(goal_msg)
    # Wait for result...
```

By decoupling the logic (Executive) from the implementation (Nav2), we made our system testable and modular. You have just built a Level 3 Autonomous Agent!
