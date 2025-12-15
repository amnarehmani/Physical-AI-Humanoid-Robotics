---
id: m1-ch5-fsm
title: "Lesson 2: State Machine Logic"
sidebar_label: "Lesson 2: State Machine"
description: "Managing the Patrol-Scan loop using a Finite State Machine."
keywords:
  - fsm
  - logic
  - python
  - state machine
  - integration
---

# Lesson 2: State Machine Logic

## 1. Introduction

A robot without a state machine is just a collection of twitchy reflexes.
If you simply write:
```python
navigator.go_to(kitchen)
scanner.scan()
navigator.go_to(bedroom)
```
Your code will fail immediately. `go_to()` is **non-blocking** (it returns instantly while the robot is still moving). The robot will try to scan *while* moving to the kitchen, and try to move to the bedroom *while* scanning. It will be chaos.

We need a **Finite State Machine (FSM)** to enforce order.
*   **State**: What am I doing right now? (e.g., NAVIGATING)
*   **Transition**: When do I stop doing it? (e.g., When distance < 0.5m)
*   **Action**: What do I do on entry/exit? (e.g., Stop motors)

## 2. Conceptual Understanding: The Patrol Cycle

The logic for our patrol robot is a loop.

```mermaid-text
       [START]
          |
          v
+------> [IDLE] <-----------------------+
|         | (Timer tick)                |
|         v                             |
|    [NAVIGATING] -------------------> [SCANNING]
|    (Action: NavigateToPose)          (Service: StartScan)
|         |                             |
|         | (Result: Success)           | (Result: Done)
|         v                             |
+---------+ ----------------------------+
```

1.  **IDLE**: Pick the next waypoint. Send Goal to Nav2. Transition to NAVIGATING.
2.  **NAVIGATING**: Wait for Nav2 Result.
    *   If Success -> Transition to SCANNING.
    *   If Failure -> Retry or Skip.
3.  **SCANNING**: Call Scanner Service. Wait for response. Transition to IDLE.

## 3. Practical Example: The Patrol Node

This node implements the FSM using a 10Hz control loop.

```python title="code/module-1/patrol_node.py"
import rclpy
from rclpy.node import Node
from enum import Enum

# Define States
class State(Enum):
    IDLE = 0
    NAVIGATING = 1
    SCANNING = 2

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        
        self.state = State.IDLE
        self.waypoints = [(2.0, 0.0, 0.0), (0.0, 2.0, 90.0), (-1.0, 0.0, 180.0)]
        self.wp_index = 0
        
        # Mocking the Navigator/Scanner interfaces for clarity
        # In real code, these would be the Classes from Lesson 1
        self.navigator = None 
        self.scanner_client = None 
        
        # The Brain Loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        # 1. State: IDLE
        if self.state == State.IDLE:
            target = self.waypoints[self.wp_index]
            self.get_logger().info(f"Patrolling to Waypoint {self.wp_index}: {target}")
            
            # Action: Send Goal
            self.navigator.send_goal(*target)
            self.state = State.NAVIGATING

        # 2. State: NAVIGATING
        elif self.state == State.NAVIGATING:
            if self.navigator.is_complete():
                result = self.navigator.get_result()
                if result == "SUCCESS":
                    self.get_logger().info("Arrived. Starting Scan.")
                    self.scanner_client.call_async(True) # Start Scan
                    self.state = State.SCANNING
                else:
                    self.get_logger().warn("Navigation Failed! Skipping.")
                    self.state = State.IDLE # Retry next WP

        # 3. State: SCANNING
        elif self.state == State.SCANNING:
            if self.scanner_client.is_done():
                self.get_logger().info("Scan Complete.")
                
                # Logic: Next Waypoint
                self.wp_index = (self.wp_index + 1) % len(self.waypoints)
                self.state = State.IDLE
```

## 4. Engineering Insights: Blocking vs Non-Blocking

*   **The Trap**: Do not use `while not navigator.is_complete(): pass` inside the loop. This blocks the thread.
*   **The Solution**: The `control_loop` runs every 0.1s. It checks status and returns immediately. This allows ROS 2 to handle incoming messages (battery status, emergency stop) in the background.

## 5. Implementing the Scanner Node

The Scanner is simple. It provides a Service. When called, it hijacks the wheels.

```python title="code/module-1/scanner_node.py"
import time
from geometry_msgs.msg import Twist

def scan_callback(self, request, response):
    # This logic blocks the Scanner Node, but that's okay for this simple example
    # Ideally, this should also be an Action or async timer loop
    msg = Twist()
    msg.angular.z = 0.5
    
    # Spin for 12 seconds
    for _ in range(120):
        self.pub.publish(msg)
        time.sleep(0.1)
        
    msg.angular.z = 0.0
    self.pub.publish(msg)
    response.success = True
    return response
```

## 6. Summary

You have built the brain.
1.  **FSM** keeps the robot from getting confused.
2.  **Control Loop** checks state at 10Hz.
3.  **Modular Nodes** (Patrol vs Scanner) keep code clean.

In the final lesson of Module 1, we will write the **Launch File** to start the Robot, Nav2, Rviz, and our Patrol system with one command.