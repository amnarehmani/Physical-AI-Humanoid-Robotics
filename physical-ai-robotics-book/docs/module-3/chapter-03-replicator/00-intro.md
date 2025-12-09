---
id: m3-ch3-intro
title: "Chapter 3: Synthetic Data Generation (Replicator)"
sidebar_label: "Introduction"
description: "Using Omniverse Replicator to train AI models."
keywords:
  - replicator
  - synthetic data
  - domain randomization
  - computer vision
---

# Chapter 3: Synthetic Data Generation (Replicator)

## Introduction

Training a robot to detect screws on a floor requires thousands of images. Taking these photos manually is slow. Labeling them (drawing bounding boxes) is expensive.
**Synthetic Data Generation (SDG)** solves this. We simulate the camera, the screw, and the floor. We know exactly where the screw is, so we get perfect labels for free.

**Omniverse Replicator** is the engine inside Isaac Sim that automates this. It can generate 10,000 labeled images in minutes, varying the lighting, texture, and position every frame.

## Learning Outcomes

By the end of this chapter, you will be able to:

1.  **Define** randomization rules (Domain Randomization) for robust AI.
2.  **Script** a Replicator graph to scatter objects.
3.  **Export** data in standard formats (KITTI, COCO, YOLO).
4.  **Visualise** the generated ground truth (segmentation masks, bounding boxes).

## Tools & Prerequisites

*   **Isaac Sim**: With Replicator extension enabled.
*   **Python**: Script editor usage.

## The Concept of Domain Randomization

If you train your AI on a perfect simulation (clean grey floor), it will fail in the real world (dirty concrete floor).
To fix this, we **randomize** the simulation.
*   **Visual Randomization**: Change floor texture (Wood, Metal, Carpet), change lighting (Red, Blue, Dark), change camera noise.
*   **Physical Randomization**: Change object mass, friction, scale.

If the AI sees 10,000 variations, it learns to ignore the "Domain" (lighting/texture) and focus on the "Task" (shape of the screw).

## Real-World Robotics Use Cases

### 1. Defect Detection
A factory needs to spot scratches on iPhones. Scratches are rare, so real data is scarce.
**Solution**: Use Replicator to generate iPhones with procedurally generated scratch textures of varying depth and angle. Train a model on this synthetic data to boost recall.

### 2. Home Robots
A Roomba needs to avoid dog poop. (You don't want to test this with real poop).
**Solution**: Spawn 1000 varieties of synthetic "obstacles" on various rug textures to train the avoidance classifier safely.

Let's write our first Replicator script.
