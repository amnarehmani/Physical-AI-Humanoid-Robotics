---
id: 002
title: update-constitution-book-rules
stage: constitution
date: 2025-12-13
surface: agent
model: gemini-2.0-flash-exp
feature: none
branch: main
user: Amna Rehman
command: /sp.constitution
labels: ["governance", "rules", "book-structure"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - null
---

## Prompt

/sp.constitution You are a senior robotics engineer, AI researcher, and technical book author.

You are expanding an existing book:
"Physical AI & Humanoid Robotics"

IMMUTABLE STRUCTURE RULES:
- DO NOT add or remove modules
- DO NOT add or remove chapters
- DO NOT add or remove lessons
- Preserve all titles and ordering
- Only EXPAND existing lessons

CONTENT DEPTH RULES:
- Each lesson must be LONG and detailed
- Explain intuition → theory → system → example → limitations
- Use real humanoid robot scenarios
- Use failure cases and trade-offs
- Avoid shallow explanations

DIAGRAM RULE:
- Include at least ONE text-based diagram per lesson
- Diagrams must be ASCII or step-flow diagrams
- Label all components clearly

STYLE RULE:
- Educational book prose
- No marketing tone
- No bullet-only sections
- Use headings and subheadings

OUTPUT FORMAT RULE:
- Output must be Docusaurus-compatible Markdown
- Use:
  - Frontmatter
  - Proper headings
  - Code blocks for diagrams
  - Callouts where useful

Acknowledge when ready to expand lessons.

## Response snapshot

Updated project constitution (.specify/memory/constitution.md) to v1.1.0.
Added new principles: "Immutable Structure", "Deep Educational Content", "Visual Explanation", "Professional Style", and "Standardized Output".

## Outcome

- ✅ Impact: Updated constitution with strict rules for book structure, content depth, visuals, and style.
- 🧪 Tests: N/A
- 📁 Files: Updated constitution.md
- 🔁 Next prompts: Start expanding content or /sp.plan
- 🧠 Reflection: User provided specific content rules masquerading as a constitution update. Codified them to ensure consistency.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
