---
id: 023
title: expand-ros2-module-1-chapter-3-lesson-2-tf2
stage: green
date: 2025-12-13
surface: agent
model: gemini-2.0-flash-exp
feature: 001-ros2-module-1
branch: main
user: Amna Rehman
command: /sp.implement
labels: ["content-expansion", "ros2", "documentation", "module-1", "chapter-3", "tf2"]
links:
  spec: specs/001-ros2-module-1/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - physical-ai-robotics-book/docs/module-1/chapter-03-custom-interfaces/02-tf2-transforms.md
tests:
 - null
---

## Prompt

/sp.implement You are expanding an existing lesson from the book:
"Physical AI & Humanoid Robotics"

VARIABLES (DO NOT CHANGE STRUCTURE):
- Module Name: {{001-ros2-module-1}}
- Chapter Name: {{Interfaces & Transform}}
- Lesson Title: {{T2F Transforms}}
- Lesson Type: {{INTRO | CORE | SYSTEM | EXAMPLE | SUMMARY}}

... [Rest of the prompt]

## Response snapshot

Re-expanded `physical-ai-robotics-book/docs/module-1/chapter-03-custom-interfaces/02-tf2-transforms.md` (TF2) following the "Adaptive Expansion Logic".
- Used the "Train" analogy to explain Relative Space.
- Added ASCII "TF Tree" diagram showing Map -> Odom -> Base hierarchy.
- Embedded `tf_listener.py` code.
- Included "Engineering Insights" on the critical difference between `/tf` and `/tf_static`.
- Maintained strict markdown formatting.

## Outcome

- ✅ Impact: Clarified one of the most confusing topics in ROS 2 (Transforms) with concrete analogies and diagrams.
- 🧪 Tests: Manual verification of file content.
- 📁 Files: physical-ai-robotics-book/docs/module-1/chapter-03-custom-interfaces/02-tf2-transforms.md
- 🔁 Next prompts: Continue with Chapter 3 Lesson 3 (Visualization).
- 🧠 Reflection: The distinction between "Static" and "Dynamic" transforms is vital for performance in complex humanoid systems.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
