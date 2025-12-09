---
id: m3-ch2-summary
title: "Chapter 2 Summary"
sidebar_label: "Summary & Quiz"
description: "Review of USD concepts."
keywords:
  - usd
  - summary
  - quiz
---

# Chapter 2 Summary

## Recap

In this chapter, we looked under the hood of Isaac Sim.
*   **USD** is not just a file format; it's a scene composition engine.
*   **Layers** allow non-destructive collaboration.
*   **References** allow us to build complex worlds from reusable assets.
*   **Prims** are the building blocks, and **Schemas** (like Physics) give them behavior.

Understanding this structure is what separates a casual user ("I drag and drop stuff") from a professional ("I generate 1000 variations of a scene programmatically").

## Future Outlook

Now that we can build the world, we need to generate data. In the next chapter, we will use **NVIDIA Replicator** to generate synthetic datasets for training Computer Vision models.

## Mini Quiz

1.  **What does USD stand for?**
    *   *Answer: Universal Scene Description.*

2.  **What is the difference between a Prim and a Property?**
    *   *Answer: A Prim is an object (Node); a Property is data on that object (Value).*

3.  **Why use a Reference instead of copying?**
    *   *Answer: To keep the scene lightweight and auto-update when the source asset changes.*

4.  **What API do you apply to make a Prim fall under gravity?**
    *   *Answer: `UsdPhysicsRigidBodyAPI`.*

5.  **Who originally developed USD?**
    *   *Answer: Pixar Animation Studios.*
