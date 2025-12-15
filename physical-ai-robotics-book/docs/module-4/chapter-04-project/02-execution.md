---
id: m4-ch4-execution
title: "Lesson 2: Executing the VLA Task"
sidebar_label: "2. VLA Task Execution"
description: "A step-by-step walkthrough of a VLA agent executing a task."
keywords:
  - vla
  - task execution
  - example
  - grounding
  - robotics
---

# Lesson 2: Executing the VLA Task

## Introduction

We have designed the VLA pipeline. Now, let's put it to the test. This lesson walks through a concrete example, demonstrating how all the modules—from speech recognition to robot action—cooperate to achieve a user's goal.

## Conceptual Understanding: A Real-World Scenario

Imagine our robot in a test environment (e.g., a simulated room in Isaac Sim).

**User Command**: "Robot, pick up the blue cube."

This simple command triggers a cascade of complex cognitive processes within the VLA agent.

## Example: Fetch the Blue Cube

Let's trace the journey of this command through our integrated VLA pipeline.

### Step 1: User Intent (ASR)

*   **User**: "Robot, pick up the blue cube."
*   **Module**: Whisper Node (Chapter 1)
*   **Process**: Audio is captured, transcribed, and published to `/speech/text`.
*   **Result**: Text string: `"Robot, pick up the blue cube."`

### Step 2: Planning & Grounding (LLM with Visual Context)

*   **Module**: LLM Planner Node (Chapter 3)
*   **Process**:
    1.  The LLM receives the command.
    2.  It uses its available tools. One tool is `vision_tool.get_objects_in_scene()`.
    3.  The LLM issues a query to the `vision_tool`: "What objects do you see, and where are they?"
    4.  **CLIP (Chapter 2) acts as the `vision_tool`**: It processes the current camera image, performs zero-shot object detection (using bounding box proposals or semantic segmentation), and identifies objects like: `{"red_sphere": bbox_coords, "blue_cube": bbox_coords, "green_cylinder": bbox_coords}`.
    5.  The LLM receives this visual observation.
    6.  **Reasoning**: "The user wants the 'blue cube'. I see a 'blue cube' at coordinates [X,Y,Z]."
    7.  **Plan Generation**: The LLM generates a grounded plan in JSON.
*   **Result**: JSON Plan:
    ```json
    [
      {"action": "navigate_to_object", "object_id": "blue_cube_1", "coordinates": [X, Y, Z]},
      {"action": "pick", "object_id": "blue_cube_1"},
      {"action": "navigate_to_user"}
    ]
    ```

### Step 3: Action Execution (Executive)

*   **Module**: Executive Node (Chapter 3)
*   **Process**:
    1.  The Executive receives the JSON plan.
    2.  It iterates through each step.
    3.  **`navigate_to_object`**: Calls the Nav2 action client (Module 3) to move the robot to the specified coordinates.
    4.  **`pick`**: Calls the manipulation action client (e.g., MoveIt) to perform the grasping sequence.
    5.  **`navigate_to_user`**: Calls Nav2 again.
*   **Result**: Robot successfully grasps the blue cube and brings it to the user.

## System Diagram: Dynamic Planning with Vision

```text
+-------------------+     +-----------------+     +-----------------+
|   USER COMMAND    | --> |   ASR (Whisper) | --> |   LLM PLANNER   |
| "Pick blue cube"  |     |                 |     |   (ReAct Loop)  |
+-------------------+     +-----------------+     +-----------------+
                                    ^    |           ^
                                    |    |           | History (Thought/Action/Obs)
                                    |    |           |
+-------------------+     +---------|----+---------+
|   ROBOT CAMERA    | --> |   CLIP PERCEPTION (Vision Tool)   |
| (Current View)    |     |   "I see blue cube at [X,Y,Z]"    |
+-------------------+     +-----------------------------------+
                                    |    |
                                    |    | (Grounded Plan: navigate, pick)
                                    v    v
                                [ EXECUTIVE ]
                                    |
                                    v
                               [ ROBOT ] (Execute)
```

## Engineering Insights: Handling Ambiguity and Failure

### Ambiguity: "Pick up *the* blue cube"
What if there are multiple blue cubes?
*   **LLM Clarification**: The LLM can ask: "Which blue cube do you mean? The one near the red sphere?" This triggers another round in the ReAct loop.
*   **Visual Cues**: The robot might use gaze or pointing to disambiguate.

### Failure: "Blue cube not found"
If CLIP returns "No blue cube detected":
*   **LLM Replanning**: The LLM might generate a new plan: "Look around," "Search under the table."
*   **Human Feedback**: If repeated attempts fail, the robot can ask the user for help.

This example illustrates the power of integrating vision and language. The robot is no longer a blind executor but an intelligent agent capable of reasoning about its environment and adapting to unforeseen circumstances.
