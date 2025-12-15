---
id: m3-ch3-annotators
title: "Lesson 2: Annotators and Writers"
sidebar_label: "Lesson 2: Writers"
description: "Capturing ground truth data."
keywords:
  - replicator
  - writer
  - annotator
  - segmentation
  - bounding box
---

# Lesson 2: Annotators and Writers

## 1. Introduction

Randomizing the world is useless if we don't capture the data.
*   **Annotators**: Virtual sensors that "see" specific truths (Color, Depth, Class ID).
*   **Writers**: Exporters that format this data for AI training (e.g., saving to disk in KITTI format).

## 2. Conceptual Understanding: The Annotator Pipeline

```text
      [ 3D Scene ]
           |
           v
      [ Render Product ] (Virtual Camera)
           |
      +----+------------------+------------------+
      |                       |                  |
   [ RGB ]              [ Semantic ]        [ BBox ]
   (Pixel Colors)       (Pixel Class IDs)   (Min/Max Coords)
      |                       |                  |
      +----------+------------+                  |
                 |                               |
            [ Writer ]                           |
                 |                               |
        ( Saves to Disk )                        |
      +----------+-----------+                   |
      |          |           |                   |
   001.png    001_seg.png   001.json             |
                                                 v
                                          [ Visualization ]
                                          (Debug Draw in GUI)
```

## 3. Implementation: Saving Data

We will extend our script to save the data.

```python
import omni.replicator.core as rep

# ... (Previous Randomization Code) ...

# 1. Create a Camera
camera = rep.create.camera(position=(0, 5, 0), rotation=(-90, 0, 0))
render_product = rep.create.render_product(camera, (1024, 1024))

# 2. Configure the Writer
# The 'BasicWriter' saves images and JSON metadata
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="_output_data",
    rgb=True,
    bounding_box_2d_tight=True,
    semantic_segmentation=True
)

# 3. Attach Writer to Camera
writer.attach([render_product])

# 4. Run
rep.orchestrator.run()
```

## 4. Engineering Insights: Semantic Segmentation

The most powerful annotator is **Semantic Segmentation**.
It produces an image where the value of every pixel is the Class ID of the object it touches.
*   Background = 0
*   Cube = 1
*   Robot = 2

**Why is this better than Bounding Boxes?**
Bounding boxes are rectangles. Robots are not. If a robot is L-shaped, the bounding box includes a lot of empty air. Segmentation is pixel-perfect. It allows the robot to learn precise grasping points.

## 5. Visualization

Before running a 10-hour generation job, **Verify** your data.
In Isaac Sim:
1.  Go to the Viewport.
2.  Click `Render` -> `Annotators`.
3.  Select `Semantic Segmentation`.

You should see your objects colored by class. If they are black, you forgot to add the `semantics=` tag when creating the object.

## 6. Summary

We have built the output end of the factory.
*   **Render Products** define the resolution.
*   **Annotators** define the data type.
*   **Writers** define the file format.

In the next lesson, we will tackle the final challenge: **Scattering**. How to drop 100 screws on the floor without them exploding.