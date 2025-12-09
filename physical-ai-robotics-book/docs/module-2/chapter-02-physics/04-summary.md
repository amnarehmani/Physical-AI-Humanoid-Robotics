---
id: m2-ch2-summary
title: "Chapter 2 Summary"
sidebar_label: "Summary & Quiz"
description: "Review of Physics and Dynamics."
keywords:
  - gazebo
  - summary
  - quiz
---

# Chapter 2 Summary

## Recap

In this chapter, we turned a hollow shell into a physical machine.
*   We calculated **Inertia Tensors**, ensuring our robot spins and accelerates realistically.
*   We defined **Collision Geometries** and tuned **Friction**, allowing our robot to grip the floor and manipulate objects.
*   We added **Joint Dynamics**, simulating the imperfections of real gearboxes and bearings.

These steps are often skipped by beginners, leading to the "It worked in simulation!" fallacy. By doing this work, you ensure your code is robust enough for the real world.

## Future Outlook

Now that our robot behaves physically, it needs to sense its environment. In Chapter 3, we will add the "eyes and ears"—Cameras, Lidars, and IMUs—to our Digital Twin.

## Mini Quiz

1.  **What happens if you define a visual mesh but no collision mesh?**
    *   *Answer: The object will pass through everything (ghost mode).*

2.  **Which parameter controls the "bounciness" or "hardness" of a contact?**
    *   *Answer: `kp` (stiffness) and `kd` (damping).*

3.  **True or False: A simple box shape is better for collision checking than a complex 3D scan.**
    *   *Answer: True. It is much faster to compute.*

4.  **If a joint oscillates wildly around its target, what dynamics parameter might help?**
    *   *Answer: Damping (it dissipates energy).*

5.  **What is the diagonal of the inertia matrix called?**
    *   *Answer: Principal Moments of Inertia (`ixx`, `iyy`, `izz`).*
