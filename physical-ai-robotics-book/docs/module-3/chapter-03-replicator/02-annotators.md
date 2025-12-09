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

## Annotators: The Sensors

An **Annotator** extracts data from the scene.
*   `rgb`: The normal image.
*   `bounding_box_2d_tight`: Boxes around objects.
*   `semantic_segmentation`: Color-coded pixels based on class.
*   `distance_to_camera`: Depth map.

## Writers: The File Exporters

A **Writer** takes the Annotator data and saves it to disk (JSON, PNG, NumPy).

```python
# Create a Writer
writer = rep.WriterRegistry.get("BasicWriter")

# Configure output directory
writer.initialize(output_dir="~/replicator_output", rgb=True, bounding_box_2d_tight=True)

# Attach to Render Product (The Camera)
render_product = rep.create.render_product("/World/Camera", (1024, 1024))
writer.attach([render_product])

# Run
rep.orchestrator.run()
```

## Visualizing Ground Truth

In Isaac Sim, switch the Viewport to "Sensors" mode.
You can select "Semantic Segmentation" from the dropdown. You should see your randomized cubes as flat colors (e.g., all cubes are red, floor is blue).
If everything is black, you forgot to set the Semantic Labels on your Prims.

## Custom Writers

The `BasicWriter` is generic. For training YOLO, you might need a custom writer that outputs `label.txt` files with normalized coordinates. Replicator allows you to write custom Python backends for this.

## End-of-Lesson Checklist

- [ ] I can configure a Writer to save RGB images.
- [ ] I can enable Bounding Box and Segmentation annotators.
- [ ] I verify the output folder contains the generated files.
- [ ] I can visualize the segmentation mask in the viewport.
