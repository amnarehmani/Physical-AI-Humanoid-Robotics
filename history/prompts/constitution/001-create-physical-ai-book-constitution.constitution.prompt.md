---
id: 001
title: Create Physical AI Book Constitution
stage: constitution
date: 2025-12-08
surface: agent
model: gemini
feature: none
branch: main
user: Amna Rehman
command: /sp.constitution
labels: ["constitution", "setup", "docusaurus", "rag"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - (none)
---

## Prompt

/sp.constitution 

Create a Constitution for my project: a Docusaurus book + integrated RAG chatbot about “Physical AI & Humanoid Robotics.” The Constitution must define global, unchanging rules for the entire project.

Include these rules:

1. **Structure**
   - All chapters follow the same format.
   - Each chapter contains 3 lessons.
   - Content flows beginner → intermediate.

2. **Writing**
   - Clear, simple explanations.
   - Consistent tone, terminology, and formatting.
   - Accurate, runnable code examples.

3. **Technical Accuracy**
   - Robotics content must follow official docs: ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA.

4. **RAG Compatibility**
   - Content must be cleanly chunkable.
   - Chatbot answers only from book text; no hallucinations.

5. **Integrity**
   - All content must be original.
   - No contradictions across chapters.

## Response snapshot

Created the project constitution defining 5 core principles: Structured Learning, Clarity & Consistency, Technical Fidelity, RAG Optimization, and Content Integrity. Set version to 1.0.0.

## Outcome

- ✅ Impact: Established project governance.
- 🧪 Tests: N/A
- 📁 Files: Updated constitution.md
- 🔁 Next prompts: Plan the book structure or setup Docusaurus.
- 🧠 Reflection: Constitution created successfully matching user constraints.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
