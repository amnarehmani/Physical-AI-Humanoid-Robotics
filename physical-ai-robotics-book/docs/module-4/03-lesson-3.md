---
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

<h2>3.1 The Goal</h2>

We will combine everything into a single system.
1.  **User**: "Bring me a soda from the table."
2.  **Whisper**: Transcribes audio.
3.  **LLM**: Generates plan `[navigate(table), pick(soda), navigate(home)]`.
4.  **Executive**: Parses JSON and calls Nav2.
5.  **Nav2**: Moves the robot in Isaac Sim.

<h2>3.2 The Executive Node</h2>

This node acts as the conductor. It reads the plan and triggers the actions one by one.

```python title="code/module-4/execution/executive_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class ExecutiveNode(Node):
    def __init__(self):
        super().__init__('executive_node')
        self.subscription = self.create_subscription(
            String, '/planning/plan', self.execute_callback, 10)

    def execute_callback(self, msg):
        plan = json.loads(msg.data)
        for step in plan:
            action = step["action"]
            target = step.get("target") or step.get("object")
            
            self.get_logger().info(f"EXECUTING: {action} -> {target}")
            
            if action == "navigate":
                # Call Nav2 Action Client here
                self.simulate_navigation(target)
            elif action == "pick":
                # Call MoveIt or Grasping Service here
                self.simulate_pick(target)
                
    def simulate_navigation(self, location):
        self.get_logger().info(f"Moving to {location}...")
        time.sleep(2) # Simulate travel time
        self.get_logger().info("Arrived.")

# ... main boilerplate ...
```

<h2>3.3 Final Exercise</h2>

1.  Launch **Isaac Sim** (Module 3).
2.  Run **Whisper Node** (Module 4, Lesson 1).
3.  Run **LLM Planner** (Module 4, Lesson 2).
4.  Run **Executive Node** (Module 4, Lesson 3).
5.  Speak a command.

Watch the logs flow:
`Heard -> Planned -> Executing -> Moving -> Done.`

You have built a voice-controlled autonomous agent.
