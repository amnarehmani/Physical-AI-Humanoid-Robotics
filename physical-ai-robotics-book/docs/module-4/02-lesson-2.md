---
id: lesson-2
title: "The Cognitive Brain"
sidebar_label: "2. Cognitive Planning"
description: "Using Large Language Models (LLMs) to plan robot actions from natural language."
keywords:
  - llm
  - gpt-4
  - planning
  - langchain
  - ros2
---

# Lesson 2: The Cognitive Brain

## Introduction

In the previous lesson, we gave the robot the ability to hear. It can now print: "I am hungry, get me a snack."
However, ROS 2 topics like `/cmd_vel` or `/goal_pose` do not accept English sentences. They require floating-point numbers.

In this lesson, we will build the **Planner Node**. This node uses a Large Language Model (LLM) to translate high-level intent into a structured sequence of actions (JSON) that the robot's body can execute.

## Conceptual Understanding: The Problem of Grounding

**Grounding** is the process of linking abstract concepts to physical reality.

*   **Abstract**: "Kitchen".
*   **Grounded**: `{"x": 5.0, "y": 2.0, "z": 0.0}`.

*   **Abstract**: "Apple".
*   **Grounded**: `Object(id=42, class="fruit", bbox=[100, 200, 50, 50])`.

An LLM like GPT-4 has read the entire internet. It knows that apples are found in kitchens. But it **does not know** where *your* kitchen is. We must provide this context in the prompt.

### The Planning Pipeline

```text
       (Text Input)
       "Get me a soda"
             |
             v
+-------------------------+
|   SYSTEM PROMPT         |
| "You are a robot..."    |
| "Locations: Kitchen..." |  <-- The Context (World Model)
+-------------------------+
             |
             v
+-------------------------+
|   LLM (Inference)       |
| (Reasoning & Logic)     |
+-------------------------+
             |
             v
+-------------------------+
|   OUTPUT PARSER         |
| (Extract JSON)          |
+-------------------------+
             |
             v
      /planning/plan
```

## Implementation: The Planner Node

We use **LangChain** to manage our prompts. LangChain allows us to easily swap models (e.g., GPT-3.5 vs GPT-4) and structure our messages.

```python title="code/module-4/planning/llm_planner.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
# We use LangChain to handle the chat history and model interaction
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage

class PlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        
        # Input: The text from Whisper
        self.subscription = self.create_subscription(
            String, '/speech/text', self.plan_callback, 10)
            
        # Output: The JSON plan for the Executive
        self.publisher_ = self.create_publisher(String, '/planning/plan', 10)
        
        # Check API Key
        if not os.getenv("OPENAI_API_KEY"):
            self.get_logger().error("OPENAI_API_KEY missing!")

        # Initialize the Model (Temperature=0 makes it more deterministic)
        self.llm = ChatOpenAI(model="gpt-4", temperature=0.0)
        
        # The "World Model" - hardcoded for this lesson
        # In a real robot, this would come from a map or database
        self.known_locations = {
            "kitchen": [5.0, 2.0],
            "bedroom": [-2.0, 3.0],
            "table": [1.5, 1.5]
        }
        
        # The System Prompt defines the "Rules of the Game"
        self.system_prompt = f"""
        You are a robotic planner for a home assistant.
        
        CAPABILITIES:
        1. navigate(target_name): Move to a location.
        2. pick(object_name): Grasp an item.
        3. place(object_name): Release an item.
        
        KNOWN LOCATIONS: {list(self.known_locations.keys())}
        
        RULES:
        - You must output a valid JSON list of steps.
        - Do not output markdown, explanations, or code blocks. Just the raw JSON.
        - If the request is impossible, return an empty list [].
        
        EXAMPLE INPUT: "Get me an apple from the kitchen"
        EXAMPLE OUTPUT:
        [
            {{"action": "navigate", "target": "kitchen"}},
            {{"action": "pick", "object": "apple"}},
            {{"action": "navigate", "target": "user"}},
            {{"action": "place", "object": "apple"}}
        ]
        """

    def plan_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Thinking about: '{command}'")
        
        messages = [
            SystemMessage(content=self.system_prompt),
            HumanMessage(content=command)
        ]
        
        try:
            # Call the LLM
            response = self.llm(messages)
            content = response.content.strip()
            
            # Parsing Safety Check
            # LLMs sometimes add ```json ... ``` wrappers. We remove them.
            if content.startswith("```json"):
                content = content[7:-3]
            elif content.startswith("```"):
                content = content[3:-3]
                
            plan_json = json.loads(content)
            
            # Validate the plan
            if not isinstance(plan_json, list):
                raise ValueError("Output is not a list")
                
            self.get_logger().info(f"Generated Plan: {json.dumps(plan_json)}")
            self.publisher_.publish(String(data=json.dumps(plan_json)))
            
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse JSON: {content}")
        except Exception as e:
            self.get_logger().error(f"Planning Error: {e}")

def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Engineering Insights: Prompt Engineering

Notice the `EXAMPLE INPUT/OUTPUT` in the system prompt. This is called **One-Shot Prompting**. By providing an example, we drastically increase the probability that the LLM follows our format.

Without the example, the LLM might reply: *"Sure! I will go to the kitchen and get the apple."*
With the example, it replies: `[{"action": "navigate", ...}]`.

### Safety and Hallucinations

LLMs hallucinate. They might output:
`{"action": "teleport", "target": "mars"}`.

To mitigate this:
1.  **Constrain the Vocabulary**: List the allowed actions explicitly.
2.  **Temperature 0**: Reduces randomness.
3.  **Post-Processing**: The code `try/except json.loads` ensures that if the LLM speaks gibberish, the robot crashes safely (throws an error) rather than moving unpredictably.

## Exercise: Test the Brain

1.  Run the Whisper Node (from Lesson 1) in one terminal.
2.  Run the Planner Node: `python3 code/module-4/planning/llm_planner.py`.
3.  Say: "Go to the kitchen and wait there."
4.  Check the output:
    ```json
    [
      {"action": "navigate", "target": "kitchen"}
    ]
    ```
5.  Say: "Bring me the remote from the bedroom."
    ```json
    [
      {"action": "navigate", "target": "bedroom"},
      {"action": "pick", "object": "remote"},
      {"action": "navigate", "target": "user"},
      {"action": "place", "object": "remote"}
    ]
    ```
