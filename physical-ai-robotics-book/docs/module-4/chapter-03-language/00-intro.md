---
id: m4-ch3-intro
title: "Chapter 3: Language Agents & Grounding"
sidebar_label: "Introduction"
description: "Using LLMs as the high-level planner for robots."
keywords:
  - llm
  - agents
  - grounding
  - prompt engineering
  - chain of thought
---

# Chapter 3: Language Agents & Grounding

## Introduction

A robot that can see "an apple" is useless if it doesn't know what to do with it.
Large Language Models (LLMs) like GPT-4 or Llama 3 have vast knowledge about the world. They know that "apples are for eating," "apples are found in kitchens," and "to get an apple, you must pick it up."

In this chapter, we convert the LLM into a **Robotic Agent**. We don't ask it to write poetry; we ask it to write **Plans**.
"User: I'm hungry." -> LLM: "1. Go to Kitchen. 2. Find Apple. 3. Pick Apple. 4. Bring to User."

However, LLMs hallucinate. They might say "Fly to the moon." Our robot cannot fly. We must **Ground** the LLM in physical reality.

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Design** prompts that force LLMs to output structured robotic commands (JSON/YAML).
2.  **Implement** a "Chain of Thought" loop where the robot Reasons, Acts, and Observes.
3.  **Ground** the LLM's output by filtering it through a list of available skills.
4.  **Connect** an LLM to a ROS 2 Action Server.

## Tools & Prerequisites

*   **OpenAI API / Ollama**: To run the LLM.
*   **LangChain**: Framework for building agents.
*   **Python**: String manipulation and API calls.

## The Grounding Problem

**Grounding** means tying a symbol (the word "Apple") to a physical reality (the pixel coordinates `x=300, y=200`).
It also means tying an action ("Pick up") to a specific code function (`arm.move_to_pose()`).
Without grounding, an LLM is a brain in a jar. With grounding, it is an embodied intelligence.

## Real-World Robotics Use Cases

### 1. SayCan (Google)
The user says "I spilled my coke."
The LLM generates options: "Vacuum it" (Impossible, liquids destroy vacuums), "Find a sponge" (Possible).
The robot calculates the "Affordance" (probability of success) for each option and chooses the sponge.

### 2. Code as Policies
Instead of outputting text steps, the LLM outputs Python code: `robot.walk_to("kitchen"); robot.pick("sponge")`. We execute this code directly on the robot.

Let's start by prompt engineering for robots.
