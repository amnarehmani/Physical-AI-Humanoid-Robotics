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

## 2.1 From Text to JSON

The robot doesn't understand "Go to the kitchen".  
It understands: `navigate_to(x=5.0, y=2.0)`.

We use an LLM (Large Language Model) to translate intent into structured JSON.

## 2.2 Prompt Engineering for Robotics

We must give the LLM a "System Prompt" defining its capabilities.

**System Prompt:**

> You are a robot planner.  
> You can perform these actions: `["navigate", "pick", "place"]`.  
> The known locations are: `{"kitchen": [5, 2], "bedroom": [-2, 3]}`.  
> Output a JSON list of actions.

**User Input:**

> "Get me an apple from the kitchen."

**Expected Output:**

```json
[
  {"action": "navigate", "target": "kitchen"},
  {"action": "pick", "object": "apple"},
  {"action": "navigate", "target": "user"},
  {"action": "place", "object": "apple"}
]


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, SystemMessage

class PlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        self.subscription = self.create_subscription(
            String, '/speech/text', self.plan_callback, 10)
        self.publisher_ = self.create_publisher(String, '/planning/plan', 10)
        
        self.llm = ChatOpenAI(model="gpt-4", temperature=0)
        # FIXED: Replaced triple quotes with parentheses and normal quotes
        self.system_prompt = (
            "You are a robot. "
            "Output valid JSON list. "
            "Actions: navigate(target), pick(item). "
            "Locations: kitchen, table."
        )

    def plan_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Planning for: {command}")
        
        messages = [
            SystemMessage(content=self.system_prompt),
            HumanMessage(content=command)
        ]
        
        response = self.llm(messages)
        plan_json = response.content
        
        self.get_logger().info(f"Plan: {plan_json}")
        self.publisher_.publish(String(data=plan_json))
