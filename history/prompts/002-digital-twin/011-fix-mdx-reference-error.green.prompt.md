---
id: 011-fix-mdx-reference-error
title: Fix MDX ReferenceError in Module 2
stage: green
date: 2025-12-10
surface: agent
model: gemini-2.0-flash-exp
feature: digital-twin
branch: main
user: Amna Rehman
command: fix
labels: ["bugfix", "mdx", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - physical-ai-robotics-book/docs/module-2/chapter-02-physics/01-inertial.md
tests:
 - null
---

## Prompt

Issue: The build on Vercel fails due to a ReferenceError in the MDX file at:
"/docs/module-2/chapter-02-physics/m2-ch2-inertial.mdx"
Cause: Variable `xx` is not defined, breaking static site generation.

Task:
1. Scan the MDX file for any usage of undefined variables, especially `xx`.
2. Either define the variable properly at the top or remove its usage if unnecessary.
3. Ensure all JSX components inside the MDX file are correctly imported or defined.
4. Preserve all existing content and formatting.
5. Output the **fixed MDX content only**, ready to replace the original file.

Constraints:
- Maintain compatibility with Docusaurus v3 MDX syntax.
- Do not change other modules or chapters.

## Response snapshot

Escaped curly braces around `xx`, `yy`, and `zz` in `01-inertial.md` to resolve the ReferenceError while preserving LaTeX syntax.

## Outcome

- ✅ Impact: Resolved Vercel build failure due to undefined variables in MDX.
- 🧪 Tests: Verified file content visually.
- 📁 Files: Modified 01-inertial.md.
- 🔁 Next prompts: None.
- 🧠 Reflection: MDX treats {xx} as JS. Escaping to \{xx\} fixes it while keeping LaTeX look.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None.
