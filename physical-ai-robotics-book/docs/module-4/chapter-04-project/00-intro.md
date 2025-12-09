---
id: m4-ch4-intro
title: "Chapter 4: Project - The VLA Agent"
sidebar_label: "Introduction"
description: "Building a fully integrated Vision-Language-Action robot."
keywords:
  - vla
  - project
  - integration
  - capstone
---

# Chapter 4: Project - The VLA Agent

## Introduction

We have the eyes (ViT/CLIP). We have the brain (LLM ReAct). Now we build the body.
In this final capstone of the book, we will build a **VLA Agent** that can perform the task: **"Tidy up the table."**

The robot will:
1.  **Scan** the table using its camera.
2.  **Identify** objects (Trash, Tools, Fruit) using CLIP.
3.  **Plan** where each object goes using an LLM.
4.  **Execute** the Pick-and-Place actions using ROS 2.

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Integrate** a Vision Service, an LLM Service, and a Control Service into one loop.
2.  **Construct** a VLA pipeline: Image -> Text Description -> Action Plan -> Robot Command.
3.  **Debug** multimodal failures (e.g., when the vision says "Apple" but the planner says "Throw away").

## Tools & Prerequisites

*   **Modules 1, 2, 3**: This project uses ROS 2, Simulation, and AI.
*   **OpenVLA / RT-2**: Concepts from state-of-the-art models.

## The Architecture

1.  **Perception Node**: Subscribes to `/camera/image_raw`. Publishes `/detected_objects` (JSON list: `[{name: "apple", pos: [0.1, 0.2]}]`).
2.  **Brain Node**: Subscribes to `/detected_objects`. Asks LLM: "I see an apple and a wrench. Where do they go?". Publishes `/action_plan`.
3.  **Control Node**: Subscribes to `/action_plan`. Calls MoveIt to execute.

## Real-World Robotics Use Cases

### 1. Household Butler
"Clean the living room." The robot sees a sock (goes to laundry), a cup (goes to kitchen), and a toy (goes to box).

### 2. Flexible Manufacturing
"Assemble the kit." The robot sees parts scattered randomly. It identifies the screw, the plate, and the driver, and plans the assembly sequence on the fly.

Let's build the Perception Node.
