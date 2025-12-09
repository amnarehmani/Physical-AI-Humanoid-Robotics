---
id: m4-ch4-execution
title: "Lesson 2: Execution and Recovery"
sidebar_label: "Lesson 2: Execution"
description: "Handling errors and closed-loop control."
keywords:
  - execution
  - recovery
  - closed-loop
  - failure
---

# Lesson 2: Execution and Recovery

## The Open Loop Failure

In Lesson 1, we assumed `robot.pick()` always succeeds. It doesn't. The object might slip.
We need **Visual Confirmation**.

## Closed-Loop Action

```python
def robust_pick(target_label):
    # 1. Try to pick
    robot.pick(target_label)
    
    # 2. Verify
    robot.move_to_camera_view()
    current_objects = detector.detect(image)
    
    # If the object is still on the table, we failed.
    if target_label in current_objects:
        return "FAILED"
    
    # If the gripper sensor detects force
    if robot.gripper.is_grasping():
        return "SUCCESS"
    
    return "DROPPED"
```

## Feeding Failure to LLM

If `robust_pick` returns `FAILED`, we send this observation back to the LLM (ReAct loop).
LLM: "Observation: Pick failed. Thought: Maybe the object is slippery. Action: Try grasping with more force."

This is the hallmark of a true **Intelligent Agent**: it persists.

## End-of-Lesson Checklist

- [ ] I can implement a verification step after an action.
- [ ] I use gripper sensors or vision to confirm grasping.
- [ ] I connect the failure state back to the LLM prompt.
- [ ] I have simulated a "slip" and watched the robot retry.
