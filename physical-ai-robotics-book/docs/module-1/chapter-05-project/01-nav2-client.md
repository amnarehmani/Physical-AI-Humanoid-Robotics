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
  - global planner
  - local planner
---

# Lesson 1: The Navigation Client

## 1. Introduction

You have a robot brain, muscles, and a body. Now, you want it to **move autonomously**.
This is not a trivial task. Moving a robot from Point A to Point B involves:
*   Localization (Where am I?)
*   Mapping (Where am I going?)
*   Global Planning (How to get there overall?)
*   Local Planning (How to avoid obstacles right now?)
*   Motor Control (How to turn the wheels?)

Manually coding this is a semester-long project. Fortunately, ROS 2 provides **Nav2 (Navigation 2)**, a complete solution. We won't *build* Nav2; we will learn to be its **Client**.

## 2. Conceptual Understanding: The Taxi Driver

**Intuition**:
Think of Nav2 as an extremely competent **Taxi Driver**.
*   You tell the driver **where to go** (Goal).
*   The driver figures out the **route**, avoids traffic, and handles the steering.
*   The driver tells you **where you are** and **how far** you have left (Feedback).
*   You can tell the driver to **stop** (Cancel).

You don't need to understand the internal combustion engine to drive a car, and you don't need to understand the internal algorithms of Nav2 to tell a robot where to go.

## 3. System Perspective: Nav2 as an Action Server

Nav2 exposes its primary interface as an Action: `NavigateToPose`. This is the perfect pattern for long-running, goal-oriented tasks.

```mermaid-text
[Your Patrol Node (Action Client)]
       |
       | (1) Goal: "Kitchen (x,y,θ)"
       |---------------------------------->
       |                                  |
       | (2) Nav2 Accepts Goal            |
       |<----------------------------------
       |                                  |
       | (3) Feedback: "Current Pose, Time Remaining"
       |<..................................
       |                                  |
       | (4) (Optional) Cancel Goal       |
       |---------------------------------->
       |                                  |
       | (5) Result: "Success / Aborted"  |
       |<----------------------------------
[ROS 2 Graph]                         [Nav2 Stack (Action Server)]
```

### The `map` Frame
All navigation goals must be expressed in the `map` frame. This is the global, stationary coordinate system of the environment. Nav2 assumes your robot is localized within this map.

## 4. Practical Example: Sending a Navigation Goal

We will create a helper class that encapsulates the complexity of talking to Nav2.

```python title="code/module-1/nav_client.py"
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
import math

class Navigator:
    def __init__(self, node: Node):
        self.node = node
        self.action_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        
        self.node.get_logger().info('Waiting for Nav2 action server...')
        self.action_client.wait_for_server()
        self.node.get_logger().info('Nav2 action server found!')

    def create_goal_pose(self, x, y, yaw_degrees) -> PoseStamped:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        # yaw_degrees needs to be converted to Quaternion
        
        yaw_rad = math.radians(yaw_degrees)
        qx, qy, qz, qw = self._euler_to_quaternion(0, 0, yaw_rad)
        goal_pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        
        return goal_pose

    def _euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def send_goal(self, x, y, yaw_degrees):
        goal_pose = self.create_goal_pose(x, y, yaw_degrees)
        self.node.get_logger().info(f'Sending goal: ({x}, {y}, {yaw_degrees}°)')
        
        # Send goal asynchronously
        self._send_goal_future = self.action_client.send_goal_async(goal_pose)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return

        self.node.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Navigation Result: {result.status}')
        # Here you would typically trigger the next state in your FSM

# Example of how to use this Navigator in a simple node
# import rclpy
# from rclpy.node import Node
#
# class MyPatrolNode(Node):
#     def __init__(self):
#         super().__init__('my_patrol_node')
#         self.navigator = Navigator(self)
#         self.navigator.send_goal(1.0, 0.0, 90.0) # Go to (1,0) facing 90 deg
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = MyPatrolNode()
#     rclpy.spin(node)
#     rclpy.shutdown()
```

## 5. Engineering Insights: Interfacing with Black Boxes

*   **Standard Interfaces**: Nav2 uses `geometry_msgs/PoseStamped` for goals. This standard ensures interoperability between different navigation algorithms.
*   **Coordinate Frames**: Always be explicit about your `frame_id`. Nav2 always operates in the `map` frame. Your robot's internal sensors might be in `base_link` or `odom`, so TF2 is crucial here.
*   **`use_sim_time`**: When running simulations, remember to set `use_sim_time` to `True` in your launch files for both Nav2 and your nodes. This ensures that time progresses at the simulation rate, not wall clock time.

## 6. Summary

You have built the most complex client so far: one that talks to an entire navigation stack.
1.  **Nav2** is your autonomous taxi driver.
2.  **`NavigateToPose`** is the Action interface.
3.  **`map` frame** is the universal coordinate system for goals.

Now that we can tell the robot where to go, we need to build the decision-making logic: The **Finite State Machine**.