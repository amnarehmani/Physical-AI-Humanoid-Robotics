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

## The Blind Planner

In Lesson 1, the robot made a full plan assuming perfect knowledge.
Real life is messy. "Navigate to Kitchen" might fail if the door is locked.
**ReAct** is a pattern where the agent thinks, acts, observes the result, and thinks again.

## The ReAct Trace

```text
Thought: User wants water. I need to go to the kitchen.
Action: navigate(kitchen)
Observation: Success. I am in the kitchen. I see [apple, sink, sponge]. I do not see water.
Thought: Water comes from the sink. I need to find a cup first.
Action: find(cup)
Observation: Found cup in cupboard.
Thought: Now I can fill it.
...
```

## Implementing the Loop

```python
history = "User: I am thirsty.\n"

while True:
    # 1. Ask LLM for next step
    response = llm.generate(history)
    action = parse_action(response)
    
    if action == "FINISH":
        break
        
    # 2. Execute Action on Robot
    result = robot.execute(action) # e.g., returns "Success" or "Door Locked"
    
    # 3. Append to history
    history += f"Action: {action}\nObservation: {result}\n"
```

## Why ReAct Matters?

It handles **Failure**.
If `navigate(kitchen)` returns `Observation: Door Locked`, the LLM sees this.
New Thought: "The door is locked. I need to find the key."
The agent adapts dynamically to the environment.

## End-of-Lesson Checklist

- [ ] I understand the difference between Open-Loop Planning and ReAct.
- [ ] I can implement a `while` loop that queries the LLM with updated history.
- [ ] I can format the robot's sensor feedback as a text "Observation".
- [ ] I have tested the agent's ability to recover from a simulated failure.

```