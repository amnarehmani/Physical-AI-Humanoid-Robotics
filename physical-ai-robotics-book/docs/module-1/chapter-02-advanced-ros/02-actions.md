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
  - asynchronous
---

# Lesson 2: Long-Running Tasks with Actions

## 1. Introduction

Topics stream data. Services answer questions. But what about **Projects**?

In humanoid robotics, most interesting tasks are projects.
*   "Navigate to the kitchen" (2 minutes).
*   "Pick up the cup" (10 seconds).
*   "Fold the shirt" (5 minutes).

If you used a Service for "Navigate to Kitchen," your client code would freeze for 2 minutes waiting for the `return` statement. During that time, you couldn't check battery levels, you couldn't see obstacles, and you couldn't tell the robot "Stop! The kitchen is on fire!"

This lesson introduces **Actions**, the most complex but powerful communication pattern in ROS 2. Actions allow us to initiate long-term goals, track their progress, and cancel them if necessary.

## 2. Conceptual Understanding: The Taxi Ride

**Intuition**:
Recall the **Taxi Ride** analogy from the introduction.
1.  **Goal**: You tell the driver, "Take me to the airport."
2.  **Acceptance**: The driver checks traffic and says, "OK, I can do that." (Or "No, I'm off duty").
3.  **Feedback**: During the ride, you look at the GPS. "10 minutes left... 5 minutes left..."
4.  **Result**: You arrive. "Here we are."
5.  **Cancellation**: Halfway there, you realize you forgot your passport. "Stop! Turn around!" The driver safely pulls over and cancels the original goal.

**Mechanism**:
An Action is not a single connection. Under the hood, it is a bundle of **Three Topics** and **Two Services**:
*   `Goal` (Service): Send target.
*   `Cancel` (Service): Abort target.
*   `Feedback` (Topic): Stream progress.
*   `Status` (Topic): Stream state (Active, Succeeded, Aborted).
*   `Result` (Service): Get final outcome.

## 3. System Perspective: The Action Interaction

Let's look at the flow of data between an Action Client (Brain) and Action Server (Navigator).

```mermaid-text
      [Client: Brain]                     [Server: Navigator]
             |                                     |
             | (1) Send Goal: "Kitchen"            |
             |------------------------------------>|
             |                                     |
             | (2) Server Accepts Goal             |
             |<------------------------------------|
             |                                     |
             |                                     | (3) Execute Control Loop
             | (4) Feedback: "Moving..."           |     (Move Wheels)
             |<....................................|
             |                                     |
             | (5) Feedback: "Halfway..."          |
             |<....................................|
             |                                     |
             | (6) Result: "Arrived"               |
             |<------------------------------------|
             |                                     |
```

## 4. Practical Example: The Fibonacci Action

Standard practice for learning Actions is the Fibonacci sequence because it simulates a task that takes time (calculating the sequence) and produces intermediate data (the sequence so far).

### 4.1 The Server
This node computes the sequence slowly (1 number per second) to simulate work.

```python title="code/module-1/fibonacci_action_server.py"
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
# We use a hypothetical 'Fibonacci' action definition for this example
# In a real workspace, you would generate this from .action file
from example_interfaces.action import Fibonacci 

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
        
        # 1. Initialize Feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        order = goal_handle.request.order

        for i in range(1, order):
            # 2. Check for Cancellation (CRITICAL)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # 3. Do Work
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[-1] + feedback_msg.partial_sequence[-2])
            
            # 4. Publish Feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            time.sleep(1)

        # 5. Succeed
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    server = FibonacciActionServer()
    rclpy.spin(server)
    rclpy.shutdown()
```

### 4.2 The Client
The client sends the goal and sets up callbacks for the journey.

```python title="code/module-1/fibonacci_action_client.py"
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

        # Send goal asynchronously
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

    def feedback_callback(self, feedback_msg):
        # Handles the stream of updates
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        # Handles the final outcome
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)
```

## 5. Engineering Insights: Preemption and Safety

Actions are the backbone of **Safe Robotics**.

*   **Preemption**: If a robot is walking to `Point A` and you send a new goal `Point B`, the Action Server can automatically "Preempt" (cancel) the first goal and switch to the second. This allows for fluid behavior switching without stopping the robot entirely.
*   **Feedback Rate**: Don't flood the network. Your control loop might run at 1000Hz, but you should only publish Feedback at 1-10Hz. Humans and UI tools don't need updates every millisecond.
*   **Watchdogs**: If the Action Server crashes, the Client needs to know. The Action interface provides connection loss detection, allowing the Brain to trigger an emergency stop if the Navigator goes silent.

## 6. Summary

Actions enable **long-running, cancellable tasks** with **continuous feedback**.
1.  **Goal**: The target.
2.  **Feedback**: The progress.
3.  **Result**: The outcome.

We now have Topics for data, Services for transactions, and Actions for goals. The final piece of the puzzle is configuration. How do we change the "Walking Speed" without recompiling code? For that, we need **Parameters**.