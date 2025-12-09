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

## The Replicator API

We import the library as `rep`.

```python
import omni.replicator.core as rep

# 1. Define the items to randomize
cube = rep.create.cube(semantics=[('class', 'cube')])
plane = rep.create.plane(scale=10)

# 2. Define the Randomization Logic
with rep.trigger.on_frame(num_frames=10):
    with cube:
        # Randomize Position (X, Z)
        rep.modify.pose(
            position=rep.distribution.uniform((-2, 0, -2), (2, 0, 2)),
            rotation=rep.distribution.uniform((0,0,0), (0,360,0))
        )
        # Randomize Color
        rep.modify.attribute(
            "inputs:diffuse_color", 
            rep.distribution.color(repeating=1)
        )
```

## Triggers

*   `rep.trigger.on_frame(num_frames=100)`: Runs for 100 frames then stops.
*   `rep.trigger.on_time(interval=1.0)`: Runs every second.

## Distributions

*   `uniform(min, max)`: Any value between min and max.
*   `normal(mean, std)`: Gaussian distribution.
*   `choice([A, B, C])`: Pick one from the list.

## Semantics

Note the `semantics=[('class', 'cube')]`. This adds a tag to the USD Prim. The "Annotators" (Lesson 2) use this tag to know "This pixel belongs to a Cube."

## End-of-Lesson Checklist

- [ ] I can write a script to create a simple shape.
- [ ] I can randomize the position and rotation of that shape.
- [ ] I can use `rep.trigger` to control how many frames are generated.
- [ ] I understand why we need to add Semantic Labels to objects.
