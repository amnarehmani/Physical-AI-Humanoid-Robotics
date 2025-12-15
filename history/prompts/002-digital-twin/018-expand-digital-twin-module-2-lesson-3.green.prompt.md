---
id: 018-expand-digital-twin-module-2-lesson-3
title: Expand Digital Twin Module 2 Lesson 3
stage: green
date: 2025-12-15
surface: agent
model: gemini-2.0-flash-exp
feature: 002-digital-twin
branch: 002-digital-twin
user: Amna Rehman
command: /sp.implement
labels: ["documentation", "ros2", "unity", "digital-twin", "bridge"]
links:
  spec: specs/002-digital-twin/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - physical-ai-robotics-book/docs/module-2/03-lesson-3.md
 - code/module-2/unity_scripts/RobotController.cs
 - specs/002-digital-twin/tasks.md
tests:
 - Manual verification of C# script validity
---

## Prompt

/sp.implement You are expanding an existing lesson from the book:
"Physical AI & Humanoid Robotics"

VARIABLES (DO NOT CHANGE STRUCTURE):
- Module Name: {{002-digital-twin}}
- Chapter Name: {{The Basics}}
- Lesson Title: {{The Digital Mirror (Unity)}}
- Lesson Type: {{INTRO | CORE | SYSTEM | EXAMPLE | SUMMARY}}

... (Rest of prompt)

## Response snapshot

Expanded `docs/module-2/03-lesson-3.md` as a **SYSTEM** lesson.
- Explained the conceptual "Split Brain" architecture (Gazebo=Physics, Unity=Graphics).
- Detailed the system perspective using ASCII diagrams for the TCP Bridge.
- Provided a full C# script implementation for `RobotController.cs` to handle coordinate mapping (ROS Right-Handed vs Unity Left-Handed).
- Added a "Real-World Example" referencing OpenAI's Rubik's Cube hand (Domain Randomization).
- Updated `specs/002-digital-twin/tasks.md` to mark Phase 5 tasks as completed.

## Outcome

- ✅ Impact: Fully expanded Lesson 3 and implemented the Unity C# bridge script.
- 🧪 Tests: No automated tests (Unity requires GUI); reliance on code correctness.
- 📁 Files: `docs/module-2/03-lesson-3.md`, `code/module-2/unity_scripts/RobotController.cs`
- 🔁 Next prompts: Module complete. Ready for final checks.
- 🧠 Reflection: Coordinate system conversion is the #1 confusing topic for students here, so I emphasized it heavily in both text and code comments.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
