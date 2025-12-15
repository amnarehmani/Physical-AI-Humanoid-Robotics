---
id: m4-ch3-prompting
title: "Lesson 1: Prompt Engineering for Robots"
sidebar_label: "Lesson 1: Prompting"
description: "Structuring inputs to get executable outputs."
keywords:
  - prompt engineering
  - json
  - few-shot
  - robotics
---

# Lesson 1: Prompt Engineering for Robots

## Introduction

If you ask ChatGPT "How do I make a sandwich?", it will give you a beautiful, poetic description.
If you ask a robot "How do I make a sandwich?", poetry is fatal. The robot needs:
1.  Move arm to (x,y,z).
2.  Close gripper.
3.  Move arm to (x', y', z').

In this lesson, we learn how to force a creative LLM to behave like a strict compiler.

## Conceptual Understanding: The Anatomy of a Prompt

A robotic prompt is not just a question. It is a strict specification document. It typically has four parts:

```text
+-------------------------------------------------------------+
|  1. PERSONA: "You are a robot controller..."                |
|  2. CONSTRAINTS: "You can ONLY use these 3 functions..."    |
|  3. WORLD STATE: "You are currently in the kitchen..."      |
|  4. FORMAT: "Output strictly in JSON format..."             |
+-------------------------------------------------------------+
```

### Structured Output (JSON)

Why JSON?
*   **Unstructured**: "Okay, first I will go to the kitchen." -> Requires complex text parsing (NLP) to execute.
*   **Structured**: `{"action": "navigate", "target": "kitchen"}` -> Trivial to parse with `json.loads()`.

## System Perspective: Determinism

LLMs are probabilistic. They roll dice to choose the next word.
For a robot, randomness is dangerous.
*   **Temperature**: Set this to `0.0`. This forces the model to choose the most likely token every time, making its behavior reproducible.
*   **Seed**: New APIs allow setting a random seed to ensure consistency across runs.

## Implementation: The Strict Planner

Let's write a Python script that ensures the robot *only* outputs valid commands.

```python
import json
import openai
import os

# Set your API Key
openai.api_key = os.getenv("OPENAI_API_KEY")

def get_robot_plan(user_command):
    system_prompt = """
    You are a robotic planner.
    
    AVAILABLE ACTIONS:
    - navigate(location): Move to a named location.
    - pick(object): Grasp an object.
    - place(location): Release the held object.
    
    LOCATIONS: [kitchen, table, bedroom]
    
    RULES:
    1. Output a valid JSON list of objects.
    2. Each object must have "action" and "arg" keys.
    3. Do not include markdown formatting (like ```json).
    4. If the request is impossible, return an empty list [].
    """
    
    response = openai.ChatCompletion.create(
        model="gpt-4",
        temperature=0.0, # CRITICAL for robotics
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_command}
        ]
    )
    
    content = response.choices[0].message.content.strip()
    
    # Validation Logic
    try:
        plan = json.loads(content)
        return plan
    except json.JSONDecodeError:
        print(f"Error: LLM output invalid JSON: {content}")
        return []

# Test
command = "Bring me the red cup from the table."
plan = get_robot_plan(command)

print(f"User: {command}")
print(f"Plan: {json.dumps(plan, indent=2)}")
```

## Advanced Technique: Chain of Thought (CoT)

For complex tasks, forcing the LLM to output JSON immediately can degrade performance. It needs "thinking time."
We can modify the prompt:
*"First, think step-by-step about the problem. Then, output the JSON plan."*

Output:
```text
THOUGHT: The user wants a cup. The cup is likely on the table. I am currently in the hallway. I need to navigate to the table first.
JSON: [{"action": "navigate", "arg": "table"}, ...]
```
We then use Regex to extract just the JSON part for the robot, while letting the LLM use the text part for reasoning.

## End-of-Lesson Checklist

- [ ] I can write a System Prompt that constrains the LLM's action space.
- [ ] I understand why `temperature=0` is critical for robotics.
- [ ] I can parse a JSON response from the LLM in Python.
- [ ] I have implemented a validator to reject impossible actions.
