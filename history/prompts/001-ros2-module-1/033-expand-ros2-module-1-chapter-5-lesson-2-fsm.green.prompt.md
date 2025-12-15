---
id: 033
title: expand-ros2-module-1-chapter-5-lesson-2-fsm
stage: green
date: 2025-12-13
surface: agent
model: gemini-2.0-flash-exp
feature: 001-ros2-module-1
branch: main
user: Amna Rehman
command: /sp.implement
labels: ["content-expansion", "ros2", "documentation", "module-1", "chapter-5", "fsm"]
links:
  spec: specs/001-ros2-module-1/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - physical-ai-robotics-book/docs/module-1/chapter-05-project/02-fsm.md
tests:
 - null
---

## Prompt

/sp.implement You are expanding an existing lesson from the book:
"Physical AI & Humanoid Robotics"

VARIABLES (DO NOT CHANGE STRUCTURE):
- Module Name: {{001-ros2-module-1}}
- Chapter Name: {{Project TurtleBot3 Patrol}}
- Lesson Title: {{State Machine }}
- Lesson Type: {{INTRO | CORE | SYSTEM | EXAMPLE | SUMMARY}}

... [Rest of the prompt]

## Response snapshot

Re-expanded `physical-ai-robotics-book/docs/module-1/chapter-05-project/02-fsm.md` (FSM) following the "Adaptive Expansion Logic".
- Introduced the "Chaos of Asynchrony" problem.
- Added ASCII FSM diagram (Idle -> Navigating -> Scanning).
- Embedded `patrol_node.py` implementing the FSM logic.
- Embedded `scanner_node.py` implementing the service logic.
- Included "Engineering Insights" on the danger of blocking loops in FSMs.
- Maintained strict markdown formatting.

## Outcome

- ✅ Impact: Clarified the central logic of the capstone project.
- 🧪 Tests: Manual verification of file content.
- 📁 Files: physical-ai-robotics-book/docs/module-1/chapter-05-project/02-fsm.md
- 🔁 Next prompts: Continue with Chapter 5 Lesson 3 (Launch).
- 🧠 Reflection: The FSM pattern is the standard for basic robot behavior; teaching it explicitly prevents "spaghetti code".

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
