---
id: m4-ch2-summary
title: "Chapter 2 Summary"
sidebar_label: "Summary & Quiz"
description: "Review of Vision Encoders."
keywords:
  - summary
  - quiz
  - vit
  - clip
---

# Chapter 2 Summary

## The Semantic Shift

In this chapter, we performed surgery on our robot's vision system. We removed the "Geometry-Only" eyes of Module 3 and installed "Semantic Eyes."

We learned that:
1.  **Vision Transformers (ViT)** process images globally using Self-Attention, capturing long-range dependencies that CNNs miss.
2.  **CLIP** aligns these visual representations with language, allowing us to query the real world using English text.

This shift allows us to build **Open-Vocabulary** robots—machines that are not limited to a fixed list of 80 objects (like COCO) but can recognize anything that can be described in words.

## The VLA Foundation

We are building a Vision-Language-Action (VLA) agent. Let's see where we stand:

```text
[ CAMERA ] -> [ ViT ] -> [ CLIP Embedding ]
                               |
                               v
[  USER  ] -> [ GPT ] -> [ Text Embedding ]
                               |
                               v
                         [ COMPARISON ] -> "Found the Apple"
```

In the next chapter, we will focus on the **Language** part. We will see how an LLM can take this "Found the Apple" signal and generate a complex multi-step plan ("Pick up the apple, wash it, and give it to the human").

## Quiz

Test your understanding of Semantic Vision.

1.  **Why do we chop images into patches for a ViT?**
    *   *Answer: Transformers take sequences (1D vectors) as input, not 2D grids. Flattening the whole image destroys local structure, so we flatten patches instead.*

2.  **What is the role of the [CLS] token?**
    *   *Answer: It is a special token that aggregates information from all other patches via attention mechanisms, serving as the final representation of the whole image.*

3.  **Explain "Contrastive Learning" in one sentence.**
    *   *Answer: Pulling matching image-text pairs closer in vector space while pushing non-matching pairs apart.*

4.  **Why do we use the prompt "A photo of a \{object\}" instead of just "\{object\}"?**
    *   *Answer: To better match the distribution of the training data (captions), which helps the model understand the context and improves accuracy.*

5.  **What is the main limitation of standard CLIP for robotics?**
    *   *Answer: It gives a global classification for the whole image but does not provide bounding boxes or coordinates (localization) by default.*
