---
id: m3-ch3-randomization
title: "Lesson 1: Domain Randomization"
sidebar_label: "Lesson 1: Randomization"
description: "Randomizing pose, color, and texture."
keywords:
  - replicator
  - randomization
  - scatter
  - python
---

# Lesson 1: Domain Randomization

## 1. Introduction

We start with the core mechanic: **Randomization**.
If we just take 1000 photos of a static cube, we have 1 data point (duplicated).
If we move the cube 1000 times, we have 1000 data points.

Replicator allows us to randomize properties (Pose, Color, Texture, Light) on every frame automatically.

## 2. Conceptual Understanding: The Graph API

Replicator is not imperative ("Do this now"). It is declarative ("Here is the rule").
We build a **Computation Graph**.
1.  **Select**: Pick the objects.
2.  **Randomize**: Define the distribution.
3.  **Trigger**: Define *when* to randomize (e.g., "Every Frame").

## 3. Implementation: Randomizing a Cube

Create `code/module-3/replicator/basic_randomizer.py`.

```python
import omni.replicator.core as rep

def randomize_cube():
    # 1. Create Assets
    # 'semantics' labels this object for the neural network
    cube = rep.create.cube(semantics=[('class', 'target_cube')])
    light = rep.create.light(intensity=500, light_type="distant")

    # 2. Define the Randomization Graph
    # Run this logic 100 times
    with rep.trigger.on_frame(num_frames=100):
        
        with cube:
            # Randomize Position: Anywhere in a 2m x 2m box
            rep.modify.pose(
                position=rep.distribution.uniform((-1, 0, -1), (1, 0, 1)),
                rotation=rep.distribution.uniform((0,0,0), (0,360,0)),
                scale=rep.distribution.uniform(0.5, 1.5)
            )
            # Randomize Color: Any RGB color
            rep.modify.attribute("inputs:diffuse_color", rep.distribution.color(repeating=1))

        with light:
            # Randomize Light Angle
            rep.modify.pose(rotation=rep.distribution.uniform((0,0,0), (360,360,360)))
            # Randomize Intensity (Dim to Bright)
            rep.modify.attribute("intensity", rep.distribution.uniform(200, 1000))

# Execute
randomize_cube()
```

## 4. Distributions

The power comes from the math distributions.
*   **`uniform(min, max)`**: Flat probability. Good for position.
*   **`normal(mean, std)`**: Bell curve. Good for size (most screw are standard size, some are defective).
*   **`choice([list])`**: Pick one. Good for textures ("Wood", "Metal", "Plastic").

## 5. Engineering Insights: Semantic Labeling

Notice `semantics=[('class', 'target_cube')]`.
This is crucial. The computer vision model doesn't know what a "Cube" prim is. It only knows labels.
We map the USD Prim to a **Class Label** ("target_cube"). The Annotator will later read this and draw a bounding box labeled "target_cube".

## 6. Summary

We have created a script that generates 100 unique variations of a cube scene.
*   **Position** changes $\rightarrow$ Net learns translation invariance.
*   **Rotation** changes $\rightarrow$ Net learns rotation invariance.
*   **Color** changes $\rightarrow$ Net learns shape focus.
*   **Lighting** changes $\rightarrow$ Net learns robustness to shadows.

In the next lesson, we will learn how to capture this data and save it as a dataset.