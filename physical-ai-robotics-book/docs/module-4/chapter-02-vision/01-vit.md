---
id: m4-ch2-vit
title: "Lesson 1: Vision Transformers (ViT)"
sidebar_label: "Lesson 1: ViT Architecture"
description: "Breaking images into patches for attention-based processing."
keywords:
  - vit
  - attention
  - patches
  - transformer
---

# Lesson 1: Vision Transformers (ViT)

## From Pixels to Patches

Convolutional Neural Networks (CNNs) look at pixels in sliding windows. They are great for textures but struggle with global context.
**Vision Transformers (ViT)** treat an image like a sentence.
1.  Take an image ($224 \times 224$).
2.  Slice it into patches ($16 \times 16$).
3.  Flatten these patches into vectors.
4.  Feed them into a Transformer Encoder (just like LLMs do with words).

## The Attention Mechanism

The key advantage is **Self-Attention**.
The top-left patch (Sky) can "attend" to the bottom-right patch (Grass) to understand the context "Landscape."
For a robot, this means the ViT can understand that a "Handle" implies a "Door," capturing the functional relationship between distant parts of an object.

## Using a Pre-trained ViT

We rarely train ViT from scratch. We use HuggingFace.

```python
from transformers import ViTImageProcessor, ViTModel
from PIL import Image
import requests

url = 'http://images.cocodataset.org/val2017/000000039769.jpg'
image = Image.open(requests.get(url, stream=True).raw)

processor = ViTImageProcessor.from_pretrained('google/vit-base-patch16-224-in21k')
model = ViTModel.from_pretrained('google/vit-base-patch16-224-in21k')

inputs = processor(images=image, return_tensors="pt")
outputs = model(**inputs)
last_hidden_states = outputs.last_hidden_state
```

The `last_hidden_states` is the "meaning" of the image.

## Why ViT for Robotics?

ViTs are robust to occlusions. If you cover half a cat, a CNN might fail. A ViT uses the visible patches to infer the missing ones, much like how humans fill in the blanks. This is crucial for robots working in cluttered environments.

## End-of-Lesson Checklist

- [ ] I understand how an image is tokenized into patches.
- [ ] I can load a pre-trained ViT using HuggingFace.
- [ ] I understand the difference between CNN local features and ViT global attention.
- [ ] I can extract the embedding vector from an image.
