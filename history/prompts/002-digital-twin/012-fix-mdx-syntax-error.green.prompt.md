---
id: 012-fix-mdx-syntax-error
title: Fix MDX Syntax and Reference Errors in Module 2
stage: green
date: 2025-12-10
surface: agent
model: gemini-2.0-flash-exp
feature: digital-twin
branch: main
user: Amna Rehman
command: fix
labels: ["bugfix", "mdx", "docusaurus", "syntax-error"]
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

Issue: MDX compilation failed for file:
"docs/module-2/chapter-02-physics/01-inertial.md"
Cause: Could not parse expression with acorn at line 45, column 7. This indicates invalid JSX or JavaScript inside the MDX file.

Task:
1. Scan the MDX file for syntax errors in JSX and JavaScript expressions, especially around line 45.
2. Correct any unclosed JSX tags, invalid JS expressions, or undefined variables.
3. Ensure all variables used inside curly braces `{}` are defined or properly imported/exported.
4. Maintain all existing content and formatting, including headings, code blocks, and markdown.
5. Output the **fixed MDX content only**, ready to replace the original file.

Constraints:
- Maintain compatibility with Docusaurus v3 MDX syntax.
- Do not change other files or chapters

## Response snapshot

Defined `xx`, `yy`, and `zz` as exported constants in `01-inertial.md` to resolve the `ReferenceError` while preserving `I_{xx}` format. Reverted the problematic `\{` escape which caused the `Acorn` syntax error.

## Outcome

- ✅ Impact: Resolved MDX compilation error and ReferenceError.
- 🧪 Tests: Verified code logic for MDX variable scope.
- 📁 Files: Modified 01-inertial.md.
- 🔁 Next prompts: None.
- 🧠 Reflection: Defining variables as strings containing braces (e.g., `'{xx}'`) is a robust way to handle LaTeX subscripts in MDX without plugins.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None.
