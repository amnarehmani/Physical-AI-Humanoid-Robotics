---
id: m1-ch2-summary
title: "Chapter 2 Summary"
sidebar_label: "Summary & Quiz"
description: "Review of Services, Actions, and Parameters."
keywords:
  - ros2
  - summary
  - quiz
---

# Chapter 2 Summary

## Recap

In this chapter, we moved from the simple broadcast world of Topics into the sophisticated transactional world of advanced ROS 2 patterns.

*   **Services** gave us the power of **Synchronization**. We can now ask questions and wait for answers, perfect for calibration, state queries, or discrete commands.
*   **Actions** gave us the power of **Time**. We can now command long-duration behaviors like navigation or manipulation, complete with feedback loops and the safety valve of cancellation.
*   **Parameters** gave us the power of **Flexibility**. We can now write code once and configure it for many different robots or environments using YAML files and Launch scripts.

Together with Topics, these form the "Golden Triangle" of ROS 2 communication. Almost every robot application is built using a combination of these three.

## Future Outlook

In the next chapter, we will look at **Custom Interfaces**. What if `int32` isn't enough? What if you need to send a message containing a "DetectedObject" with a class ID, confidence score, and 3D bounding box? We will learn to define our own data types.

## Mini Quiz

Test your knowledge of Chapter 2.

1.  **True or False**: A Service Client must always block the main thread while waiting for a response.
    *   *Answer: False. Async calls are preferred to keep the node reactive.*

2.  **Which communication pattern is best for "Telling a robot to drive to a charging station"?**
    *   A) Topic
    *   B) Service
    *   C) Action
    *   *Answer: C) Action. It takes time, needs feedback, and might need to be cancelled.*

3.  **What happens to a Parameter value when you restart a node (without a launch file/YAML)?**
    *   *Answer: It resets to the default value defined in the code.*

4.  **In an Action, which channel goes from Server to Client?**
    *   A) Goal
    *   B) Feedback
    *   C) Result
    *   *Answer: B (Feedback) and C (Result).*

5.  **Why do we use Launch files?**
    *   *Answer: To start multiple nodes simultaneously and apply configuration parameters automatically.*
