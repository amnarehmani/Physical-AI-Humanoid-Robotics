---
id: m4-ch2-clip
title: "Lesson 2: Zero-Shot Detection with CLIP"
sidebar_label: "Lesson 2: CLIP"
description: "Matching images to text descriptions."
keywords:
  - clip
  - zero-shot
  - openai
  - multimodal
---

# Lesson 2: Zero-Shot Detection with CLIP

## The Alignment Problem

We have a powerful Image Encoder (ViT) and a powerful Text Encoder (GPT). But they speak different languages.
**CLIP** trains them together on 400 million (Image, Text) pairs.
It forces the vector of "Image of a dog" to be close to the vector of "Text: Dog".

## Implementing CLIP

```python
import torch
import clip
from PIL import Image

device = "cuda" if torch.cuda.is_available() else "cpu"
model, preprocess = clip.load("ViT-B/32", device=device)

image = preprocess(Image.open("robot_view.jpg")).unsqueeze(0).to(device)
text = clip.tokenize(["a wrench", "a screwdriver", "a hammer"]).to(device)

with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text)
    
    # Calculate Similarity (Dot Product)
    logits_per_image, logits_per_text = model(image, text)
    probs = logits_per_image.softmax(dim=-1).cpu().numpy()

print("Label probs:", probs)
```

## Robot Perception

Imagine a robot looking at a table.
1.  Camera takes a photo.
2.  User asks: "Pick up the hammer."
3.  We pass the image and the text ["hammer", "table", "wall"] to CLIP.
4.  If "hammer" has the highest probability, the robot knows the object is present.
5.  (Advanced): We crop the image into sliding windows and check which window matches "hammer" best to find its location.

## Zero-Shot Power

We never trained the robot on "Hammer." CLIP learned it from the internet. This allows us to deploy robots that can recognize *anything* without retraining.

## End-of-Lesson Checklist

- [ ] I can load the CLIP model.
- [ ] I can compare an image against a list of text prompts.
- [ ] I understand what "Zero-Shot" means in this context.
- [ ] I can interpret the softmax probability output.
