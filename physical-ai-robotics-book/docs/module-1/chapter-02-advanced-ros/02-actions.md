---
id: m1-ch2-actions
title: "Lesson 2: Long-Running Actions"
sidebar_label: "Lesson 2: Actions"
description: "Managing complex goals with Goal, Feedback, and Result."
keywords:
  - ros2
  - actions
  - navigation
  - manipulation
  - python
---

# Lesson 2: Long-Running Tasks with Actions

## The Goal-Oriented Robot

Services are great for "quick questions," but robotics is full of "long projects."
*   "Navigate to the kitchen" (Takes 2 minutes).
*   "Fold the laundry" (Takes 10 minutes).
*   "Scan the room" (Takes 30 seconds).

If you used a Service for these, your client would hang for 2 minutes, unable to do anything else. Worse, you couldn't tell the robot "Stop! That's the wrong room!" once the request was sent.

Enter the **Action**.

## Anatomy of an Action

An Action is built on top of three separate communication channels (Topics and Services under the hood):

1.  **Goal**: The client requests a task. (e.g., `Target: Kitchen`).
2.  **Feedback**: The server reports progress periodically. (e.g., `Distance: 5m`, `Distance: 4m`...).
3.  **Result**: The final outcome. (e.g., `Arrived`).

Crucially, Actions support **Cancellation**. The client can say "Cancel Goal" at any time, and the server handles the graceful stop.

![diagram](pathname://placeholder-ros2-action-structure)
*Figure 2.2: The Action Client-Server interaction loop.*

## Implementing an Action Server

We will implement a Fibonacci Action. Why Fibonacci? It simulates a long calculation that generates a sequence (Feedback) and a final result (Result).

### Step 1: The Action Definition
Typically defined in a `.action` file:
```text
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```
(For this lesson, we assume `action_tutorials_interfaces` or similar exists, or we define it abstractly).

### Step 2: The Server Code

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
# Assuming a custom interface, or we can use a standard one for demo
from example_interfaces.action import Fibonacci # Hypothetical standard

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self.get_logger().info("Fibonacci Action Server Ready")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # The Goal Request
        order = goal_handle.request.order

        for i in range(1, order):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Compute next number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[-1] + feedback_msg.partial_sequence[-2])
            
            # Publish Feedback
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate work
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result
```

## Implementing an Action Client

The Client is more complex than a Service Client because it must handle the Feedback callbacks separately from the Result callback.

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')
```

## Real-World: Navigation 2 (Nav2)

The most famous use of Actions in ROS 2 is **Nav2**.
When you give a robot a "2D Nav Goal" in Rviz, you are actually sending an Action Goal (`NavigateToPose`).
*   **Goal**: Coordinates (x, y, theta).
*   **Feedback**: Current position, Time remaining.
*   **Result**: Success/Failure.
*   **Cancel**: User presses "Stop".

This standard interface allows *any* robot (drone, car, humanoid) to be controlled by the same Navigation software.

## End-of-Lesson Checklist

- [ ] I can describe the three components of an Action (Goal, Feedback, Result).
- [ ] I understand how Cancellation makes Actions safer for physical robots.
- [ ] I have examined the code structure for an Action Server.
- [ ] I recognize that Navigation stacks heavily rely on Actions.

Next, we will look at how to configure these nodes dynamically without rewriting code.
