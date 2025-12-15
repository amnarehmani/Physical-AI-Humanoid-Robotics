---
id: m3-ch3-scatter
title: "Lesson 3: Advanced Scatter"
sidebar_label: "Lesson 3: Scatter"
description: "Scattering debris and objects naturally."
keywords:
  - replicator
  - scatter
  - physics
  - collision
---

# Lesson 3: Advanced Scatter

## 1. Introduction

Placing one object is easy. Placing 100 objects is hard.
If you simply randomize `x,y` coordinates for 100 screws using `uniform()`, they will overlap. In the real world, matter cannot occupy the same space. If your dataset shows intersecting screws, the AI learns impossible physics.

We need a **Scatter Algorithm**.

## 2. Conceptual Understanding: Poisson Disk Sampling

To place objects naturally but randomly, we use sampling algorithms that respect:
1.  **Bounds**: Stay on the floor.
2.  **Collisions**: Don't touch each other.
3.  **Density**: How crowded is the scene?

```text
      [ Attempt Spawn at X,Y ]
               |
      [ Check Overlap? ] --(Yes)--> [ Retry ]
               | (No)
               v
        [ Place Object ]
               |
        [ Count < N? ] --(Yes)--> [ Loop ]
```

## 3. Implementation: Scatter 2D

Replicator provides a high-level function `scatter_2d`.

```python
import omni.replicator.core as rep

def scatter_screws():
    # 1. The Surface
    floor = rep.create.plane(scale=5)
    
    # 2. The Objects
    # Load 50 instances of the screw asset
    screws = rep.create.from_usd("assets/screw.usd", count=50)

    # 3. The Logic
    with rep.trigger.on_frame():
        rep.randomizer.scatter_2d(
            screws,
            surface=floor,
            check_collisions=True, # Critical!
            no_coll_prims=[],
            rotation=rep.distribution.uniform((-90, 0, 0), (-90, 360, 0)) # Screws lying flat
        )
```

## 4. Physics-Based Settling

Usually, `scatter_2d` is purely geometric. For pile-ups (screws on top of screws), we need Physics.
Replicator supports **Simulation Context**. You spawn objects high in the air, run physics for 60 frames (let them fall), *then* trigger the Writer.

## 5. Engineering Insights: Headless Mode

Running this GUI consumes GPU. When generating 100,000 images, we run **Headless**.
```bash
./isaac-sim.sh --no-window --python code/module-3/replicator/generate_dataset.py
```
This runs the simulation without rendering the GUI window, maximizing performance for the Annotators.

## 6. Summary

We have mastered Synthetic Data Generation.
*   **Domain Randomization**: Varies the look.
*   **Writers**: Saves the truth.
*   **Scatter**: Creates the clutter.

This dataset is now ready to train a YOLO or MaskRCNN model. This concludes Module 3. We have built the Brain (AI) and the Environment (Sim).
In the final Module 4, we will integrate Language.