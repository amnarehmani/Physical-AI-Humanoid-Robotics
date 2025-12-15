---
id: m4-ch3-react
title: "Lesson 2: ReAct (Reason + Act)"
sidebar_label: "Lesson 2: ReAct Loop"
description: "The feedback loop of observation and action."
keywords:
  - react
  - reasoning
  - feedback loop
  - agents
---

# Lesson 2: ReAct (Reason + Act)

## Introduction

In Lesson 1, we built a **Linear Planner**. The user said "Get the apple," and the LLM output a full list of steps.
This works if the world is perfect. But what if the apple isn't on the table? What if the door is locked?
A linear planner fails because it assumes it knows everything at the start.

**ReAct (Reason + Act)** is a paradigm where the robot figures things out step-by-step. Instead of a full plan, it generates just *one* action, executes it, looks at the result, and then decides what to do next.

## Conceptual Understanding: The Loop

The ReAct loop has three components:

1.  **Thought**: The LLM analyzes the current state and decides what info it needs.
2.  **Action**: The robot performs a specific task (e.g., `open_door`, `scan_table`).
3.  **Observation**: The environment returns feedback (e.g., "Door Opened", "Apple found").

```text
       (Start)
          |
          v
+----[ LLM (Brain) ] <-------+
|  "I need to find the cup"  |
|  "I will look on table"    |
+----------------------------+
          |
     (Action: scan)
          |
          v
+----[ Robot (Body) ] -------+
|  Executes scan...          |
|  Sensor: "No cup found"    |
+----------------------------+
          |
     (Observation)
          |
          v
    [ History Buffer ] ------+
```

## Implementation: The ReAct Agent

We will build a simple ReAct loop in Python. We will "mock" the environment for clarity.

```python
import openai

# Mock Environment
world_state = {
    "kitchen": ["apple", "fridge"],
    "bedroom": ["bed", "book"],
    "fridge": ["coke"] # The coke is INSIDE the fridge
}
current_location = "kitchen"

def execute_action(action_str):
    global current_location
    cmd, arg = action_str.split("(")
    arg = arg.strip(")")
    
    if cmd == "navigate":
        current_location = arg
        return f"Moved to {arg}."
    elif cmd == "look":
        # Returns list of visible items
        items = world_state.get(current_location, [])
        return f"I see: {items}"
    elif cmd == "open":
        # Check if item is a container
        if arg in world_state:
            contents = world_state[arg]
            return f"Opened {arg}. Inside is: {contents}"
        return "Cannot open that."
    return "Unknown action."

# The ReAct Loop
history = """
SYSTEM: You are a robot. 
Available Actions: navigate(loc), look(), open(item).
Goal: Find the coke.
"""

for i in range(5): # Limit steps to prevent infinite loops
    print(f"\n--- Step {i} ---")
    
    # 1. THOUGHT (Call LLM)
    prompt = history + "\nAssistant:"
    response = openai.ChatCompletion.create(
        model="gpt-4", messages=[{"role": "user", "content": prompt}]
    ).choices[0].message.content
    
    print(f"LLM: {response}")
    
    # Parse Action (Simplified for demo)
    # Assumes LLM outputs: "Thought: ...\nAction: cmd(arg)"
    if "Action:" not in response:
        print("Done or Failed.")
        break
        
    action_line = response.split("Action:")[-1].strip()
    
    # 2. ACTION (Execute)
    observation = execute_action(action_line)
    print(f"Observation: {observation}")
    
    # 3. UPDATE HISTORY
    history += f"\nAssistant: {response}\nSystem: Observation: {observation}"
```

## Trace Analysis

If we run the code above, we get this trace:

1.  **Thought**: "I am in the kitchen. I should check if the coke is visible."
2.  **Action**: `look()`
3.  **Observation**: "I see: [apple, fridge]"
4.  **Thought**: "I don't see the coke. It might be inside the fridge."
5.  **Action**: `open(fridge)`
6.  **Observation**: "Opened fridge. Inside is: [coke]"
7.  **Thought**: "I found the coke!"

This dynamic behavior—searching inside containers—is impossible with a simple linear planner.

## End-of-Lesson Checklist

- [ ] I understand the difference between Open-Loop Planning and ReAct.
- [ ] I can implement a `while` loop that queries the LLM with updated history.
- [ ] I can format the robot's sensor feedback as a text "Observation".
- [ ] I have tested the agent's ability to recover from a simulated failure.