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

## Introduction

For a decade (2012-2020), Computer Vision was dominated by **Convolutional Neural Networks (CNNs)** like ResNet and VGG. These networks work by sliding a small window (kernel) over the image, detecting edges, then textures, then shapes.

This approach has a flaw: **Local Inductive Bias**. A CNN struggles to connect distant parts of an image. If a robot sees a tail in the top-left corner and a paw in the bottom-right, a standard CNN needs many layers to realize they belong to the same cat.

The **Vision Transformer (ViT)** changed everything. Instead of sliding windows, it treats an image like a sequence of words, allowing every part of the image to "pay attention" to every other part instantly.

## Conceptual Understanding: The Architecture

How do we feed a 2D image into a 1D Transformer?

### 1. Patchify
We slice the image into fixed-size squares (patches). Standard ViT uses $16 \times 16$ pixel patches.
A $224 \times 224$ image becomes a sequence of $196$ patches.

### 2. Linear Projection
We flatten each patch (pixels) into a long vector and map it to a specific dimension $D$ (e.g., 768). This is now our "Word Embedding."

### 3. Position Embeddings
Since a Transformer has no inherent sense of order, we simply *add* a learned vector to each patch that says "I am the top-left patch" or "I am the center patch."

### 4. The [CLS] Token
We prepend a special learnable vector called the **Class Token ([CLS])**. As the information flows through the network layers, this token aggregates information from all other patches. By the end, the [CLS] token represents the *entire image*.

### Architecture Diagram

```text
       [ Class Probability / Embedding ]
                     ^
                     | (MLP Head)
           [ Transformer Encoder ]
           [ Transformer Encoder ] ... x12 Layers
                     ^
                     |
[CLS] + [Patch 1] + [Patch 2] ... + [Patch N]
  ^        ^           ^               ^
  |        |           |               |
(Learned) (Flatten) (Flatten)       (Flatten)
           |           |               |
      [ Image 1 ] [ Image 2 ] ... [ Image N ]
```

## System Perspective: Self-Attention

Why is this better for robotics?
**Global Context**.
In a cluttered room, a robot needs to know that the "cable" on the floor connects to the "lamp" on the table. A CNN might see them as two separate objects. A ViT, through its **Self-Attention** mechanism, can learn the relationship between the pixels of the cable and the pixels of the lamp, even if they are far apart in the image.

## Implementation: Using HuggingFace

We don't need to write the math from scratch. We can use the `transformers` library.

```python
from transformers import ViTImageProcessor, ViTModel
from PIL import Image
import requests
import torch

# 1. Load an image
url = 'http://images.cocodataset.org/val2017/000000039769.jpg'
image = Image.open(requests.get(url, stream=True).raw)

# 2. Load Pre-trained Model
# "google/vit-base-patch16-224" means: Base size, 16x16 patches, 224x224 input
processor = ViTImageProcessor.from_pretrained('google/vit-base-patch16-224-in21k')
model = ViTModel.from_pretrained('google/vit-base-patch16-224-in21k')

# 3. Preprocess
inputs = processor(images=image, return_tensors="pt")

# 4. Forward Pass
with torch.no_grad():
    outputs = model(**inputs)

# 5. Extract Embeddings
# last_hidden_state shape: [Batch_Size, Sequence_Length, Hidden_Dim]
# Sequence_Length = 197 (196 patches + 1 CLS token)
last_hidden_states = outputs.last_hidden_state

# The [CLS] token is at index 0
image_embedding = last_hidden_states[0, 0, :]

print(f"Embedding Shape: {image_embedding.shape}")
# Output: torch.Size([768])
```

This 768-dimensional vector is the "Semantic Fingerprint" of the image.

## End-of-Lesson Checklist

- [ ] I understand how an image is chopped into patches.
- [ ] I understand the role of the [CLS] token.
- [ ] I can explain why ViT has better global context than a CNN.
- [ ] I have run the code to extract an embedding vector.
