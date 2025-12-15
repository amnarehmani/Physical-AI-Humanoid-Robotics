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

## Introduction

We have a Vision Transformer that converts images to vectors. We have Language Models that convert text to vectors. But if we run them separately, the vector for "Dog" (image) and "Dog" (text) will be completely different numbers. They live in different "Latent Spaces."

**CLIP (Contrastive Language-Image Pre-training)** solves this. It forces these two spaces to align. It creates a universal language where images and text are interchangeable.

## Conceptual Understanding: Contrastive Learning

OpenAI scraped 400 million pairs of (Image, Caption) from the internet. They didn't label them manually. They just assumed the caption matched the image.

The training goal is simple:
1.  Take a batch of $N$ images and $N$ texts.
2.  Maximize the cosine similarity between the correct pairs (Image 1, Text 1).
3.  Minimize the cosine similarity between incorrect pairs (Image 1, Text 2...N).

This simple objective forces the model to learn deep semantic concepts. It learns that "spotted" applies to both "leopard" and "dalmatian."

### Architecture Diagram

```text
[ Image ] --> [ Image Encoder (ViT) ] --> [ Vector I ]
                                               |
                                          (Dot Product) --> Similarity Score
                                               |
[ Text  ] --> [ Text Encoder (Tx)   ] --> [ Vector T ]

Inference:
1. Encode Image once.
2. Encode 100 possible text labels.
3. Find which text vector is closest to the image vector.
```

## Implementation: The Semantic Detector

Let's build a script that lets a robot identify tools on a workbench without ever training it on tool datasets.

```python
import torch
import clip
from PIL import Image

# 1. Load the Model
device = "cuda" if torch.cuda.is_available() else "cpu"
# "ViT-B/32" = ViT Base with 32x32 patches
model, preprocess = clip.load("ViT-B/32", device=device)

# 2. Prepare the Image
image = preprocess(Image.open("workbench_view.jpg")).unsqueeze(0).to(device)

# 3. Prepare the Text Labels
# We use a "Prompt Template" to help the model understand the context
labels = ["hammer", "screwdriver", "wrench", "tape measure"]
text_inputs = torch.cat([clip.tokenize(f"a photo of a {c}") for c in labels]).to(device)

# 4. Inference
with torch.no_grad():
    image_features = model.encode_image(image)
    text_features = model.encode_text(text_inputs)

    # 5. Calculate Similarity
    # Normalize features first!
    image_features /= image_features.norm(dim=-1, keepdim=True)
    text_features /= text_features.norm(dim=-1, keepdim=True)
    
    # Similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)
    similarity = (100.0 * image_features @ text_features.T).softmax(dim=-1)
    values, indices = similarity[0].topk(1)

# 6. Output
print(f"\nPrediction: {labels[indices[0]]}")
print(f"Confidence: {values[0].item():.2f}%")
```

### Prompt Engineering for Vision
Notice we wrote `f"a photo of a {c}"` instead of just `{c}`.
This is a hack called **Prompt Ensembling**. CLIP was trained on captions like "A photo of a dog," not just the word "Dog." By adding the sentence structure, we align our input closer to the training data, often boosting accuracy by 5-10%.

## System Perspective: Spatial CLIP

Standard CLIP gives one label for the whole image. But robots need to know *where* the object is.
How do we do Object Detection?

1.  **Sliding Window**: Crop the image into many small squares. Run CLIP on each square. This is slow.
2.  **Grad-CAM**: Look at the attention map of the ViT to see which pixels "activated" for the word "Hammer."
3.  **Owl-ViT**: A newer Google model based on CLIP specifically designed to output Bounding Boxes.

## End-of-Lesson Checklist

- [ ] I understand how Contrastive Learning aligns image and text spaces.
- [ ] I can write a Python script to classify an image against a custom list of words.
- [ ] I understand why we use the prompt "a photo of a...".
- [ ] I know the difference between Image-Level Classification and Object Detection.
