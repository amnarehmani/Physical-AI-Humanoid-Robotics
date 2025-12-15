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

In Chapter 2, we gave our robot **Semantic Vision**. It can now look at a scene and say, "There is an apple at (5,2)."
But what does it *do* with that apple?

Large Language Models (LLMs) like GPT-4 possess vast intelligence. They know apples are edible, they know gravity makes things fall, and they know social norms (don't throw apples at people). However, an LLM is a **Brain in a Jar**. It lives on a server, disconnected from the physical world. It has no hands, no eyes, and no concept of "here" or "now."

In this chapter, we bridge the gap between the ethereal world of language and the rigid world of robotics. We turn the LLM into an **Agent**.

## Conceptual Understanding: The Symbol Grounding Problem

One of the oldest problems in AI is the **Symbol Grounding Problem**.
To an LLM, the word "Apple" is just a statistical vector close to "Fruit." It has no sensory experience of redness, crunchiness, or weight.
**Grounding** is the process of attaching these abstract symbols to physical reality.

1.  **Perceptual Grounding**: Linking the word "Apple" to the cluster of red pixels in the camera feed.
2.  **Action Grounding**: Linking the verb "Pick up" to the specific joint torques required to close the gripper.

## System Perspective: Affordances

A key concept in robotics is **Affordance**. An affordance is an action that is *possible* given the current environment and hardware.
*   A "Cup" affords "Grasping."
*   A "Wall" does *not* afford "Walking through."

An ungrounded LLM might generate a plan: "1. Walk through the wall. 2. Levitate the cup."
Our job is to filter the LLM's imagination through the reality of the robot's affordances.

### Architecture Diagram

```text
UNEMBODIED LLM (Chatbot)
User: "I spilled my drink."
LLM: "Oh no! You should grab a paper towel." (Helpful advice, but no action)

EMBODIED AGENT (Robot)
User: "I spilled my drink."
LLM Thought: "The user needs help. I see a sponge. I can grasp the sponge."
Robot Action: [Navigate(Kitchen), Grasp(Sponge), Navigate(User)]
```

## Real-World Robotics Use Case: Google's SayCan

In 2022, Google Robotics released a landmark paper called **SayCan**. It formalized this problem mathematically.
The system calculates two probabilities for every possible action:
1.  **Say (Relevance)**: How likely is this action to be useful? (Calculated by LLM).
2.  **Can (Affordance)**: How likely is the robot to successfully do it? (Calculated by a Value Function).

**Score = Say × Can**

If the user asks for "clean up," the LLM might say "Vacuuming" is highly relevant (High Say). But if the robot sees a liquid spill, the Value Function says "Vacuuming will break me" (Low Can). The combined score drops, and the robot chooses "Sponge" instead.

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

Let's begin by teaching the LLM to speak "Robot."
