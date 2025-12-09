---
id: m1-ch5-nav2
title: "Lesson 1: The Navigation Client"
sidebar_label: "Lesson 1: Navigation Client"
description: "Sending goals to the Nav2 stack programmatically."
keywords:
  - nav2
  - action client
  - python
  - navigate_to_pose
---

# Lesson 1: The Navigation Client

## Understanding Nav2

Nav2 is a massive stack of nodes (Planner, Controller, Recoveries, Map Server).
Fortunately, we only need to talk to one interface: The `NavigateToPose` Action Server.

## The Action Interface

Action Type: `nav2_msgs/action/NavigateToPose`

*   **Goal**: `geometry_msgs/PoseStamped` (Target position).
*   **Feedback**: `current_pose`, `navigation_time`, `distance_remaining`.
*   **Result**: `Success`, `Aborted`, or `Canceled`.

## The Python Client

We will create a helper class to manage navigation.

```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class Navigator:
    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    
    def go_to(self, x, y, theta):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        # Todo: Convert theta to Quaternion (x,y,z,w)
        
        self.client.wait_for_server()
        return self.client.send_goal_async(goal)
```

## Quaternion Math

ROS 2 uses Quaternions for rotation, not Euler angles (Roll, Pitch, Yaw).
We need a helper to convert:

```python
import math
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)
```

## Testing

1.  Launch TurtleBot3 Simulation: `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`.
2.  Launch Nav2: `ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True`.
3.  Run your client node. The robot should move!

## End-of-Lesson Checklist

- [ ] I can import `NavigateToPose` from `nav2_msgs`.
- [ ] I have a helper function to convert Euler angles to Quaternions.
- [ ] I can send a goal to `(x=2.0, y=0.0)` and watch the simulated robot move.
- [ ] I understand that the goal frame must be `map`.
