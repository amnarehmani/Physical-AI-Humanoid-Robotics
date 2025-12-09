---
id: m4-ch3-summary
title: "Chapter 3 Summary"
sidebar_label: "Summary & Quiz"
description: "Review of Language Agents."
keywords:
  - summary
  - quiz
  - llm
  - react
---

# Chapter 3 Summary

## Recap

In this chapter, we turned the robot into a thinking machine.
*   **Prompt Engineering** allowed us to constrain the infinite creativity of the LLM into strict robotic commands.
*   **ReAct** gave the robot a feedback loop, allowing it to adapt, recover from errors, and explore its environment.
*   **Grounding** ensured that the robot's thoughts remained tied to its physical capabilities.

This is the **L** (Language) in **VLA**.

## Future Outlook

In the final chapter, we will combine **Vision** (Chapter 2) and **Language** (Chapter 3) into a single **VLA Agent**. The robot will look at a scene, describe it, plan an action, and execute it.

## Mini Quiz

1.  **What is "Grounding" in the context of robotics?**
    *   *Answer: Linking abstract concepts (words) to physical objects (pixels) and executable actions (code).*

2.  **What is a System Prompt?**
    *   *Answer: An initial instruction given to the LLM to define its persona and constraints.*

3.  **In the ReAct pattern, what comes after an Action?**
    *   *Answer: An Observation (feedback from the environment).*

4.  **Why is JSON a good output format for LLMs in robotics?**
    *   *Answer: It is structured, easy to parse programmatically, and supports nested data.*

5.  **What happens if you don't validate the LLM's output?**
    *   *Answer: The robot might crash by trying to execute a hallucinated or impossible command.*
