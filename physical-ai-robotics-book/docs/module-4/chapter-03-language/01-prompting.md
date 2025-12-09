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

## The System Prompt

You must define the robot's persona and constraints.

```text
SYSTEM: You are a helper robot named 'Botty'. 
You control a mobile base with a gripper.
You can ONLY perform these actions: [navigate(location), pick(object), place(location)].
Do not invent actions like 'jump' or 'run'.
Output your plan as a JSON list.
```

## Few-Shot Learning

LLMs learn best by example. Give it examples of correct behavior.

```text
User: "Bring me the red cup."
Assistant: [
  {"action": "navigate", "arg": "kitchen"},
  {"action": "pick", "arg": "red_cup"},
  {"action": "navigate", "arg": "user"},
  {"action": "place", "arg": "table"}
]
```

## Parsing the Output

In Python, we take the string response and parse it.

```python
import json
import openai

response = openai.ChatCompletion.create(...)
plan_str = response.choices[0].message.content
plan = json.loads(plan_str)

for step in plan:
    if step['action'] == 'navigate':
        robot.navigate(step['arg'])
    elif step['action'] == 'pick':
        robot.pick(step['arg'])
```

## Hallucination Checks

Even with good prompts, the LLM might output `{"action": "teleport"}`.
Your code must validate every step against a "Skill Library." If an invalid action is found, throw an error and ask the LLM to replan.

## End-of-Lesson Checklist

- [ ] I can write a System Prompt that constrains the LLM's action space.
- [ ] I can provide Few-Shot examples to guide the output format.
- [ ] I can parse a JSON response from the LLM in Python.
- [ ] I have implemented a validator to reject impossible actions.
