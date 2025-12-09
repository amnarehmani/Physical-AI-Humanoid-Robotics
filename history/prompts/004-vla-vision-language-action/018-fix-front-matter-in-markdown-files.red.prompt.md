---
id: 018
title: Fix Front Matter in Markdown Files
stage: red
date: 2025-12-08
surface: agent
model: gemini
feature: vla-vision-language-action
branch: 004-vla-vision-language-action
user: Amna Rehman
command: /sp.implement
labels: ["fix", "front-matter", "docusaurus"]
links:
  spec: specs/004-vla-vision-language-action/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - physical-ai-robotics-book/docs/module-3/00-intro.md
 - physical-ai-robotics-book/docs/module-3/01-lesson-1.md
 - physical-ai-robotics-book/docs/module-3/02-lesson-2.md
 - physical-ai-robotics-book/docs/module-3/03-lesson-3.md
 - physical-ai-robotics-book/docs/module-3/04-summary.md
 - physical-ai-robotics-book/docs/module-4/00-intro.md
 - physical-ai-robotics-book/docs/module-4/01-lesson-1.md
 - physical-ai-robotics-book/docs/module-4/02-lesson-2.md
 - physical-ai-robotics-book/docs/module-4/03-lesson-3.md
 - physical-ai-robotics-book/docs/module-4/04-summary.md
tests:
 - (none)
---

## Prompt

Scan all Markdown files in my Docusaurus book under **/docs/module-3/** and **/docs/module-4/**. 

Tasks:
- Detect invalid YAML front matter (unescaped colons, missing quotes, misaligned keys)
- Wrap all values containing special characters (e.g., ":") in double quotes
- Ensure `id`, `title`, `sidebar_label`, `description`, and `keywords` are present and properly formatted
- Remove any null characters or invalid whitespace at the end of files
- Keep all existing content intact
- Output corrected Markdown files ready to build in Docusaurus

## Response snapshot

Scanned and updated front matter in all markdown files for Modules 3 and 4.
- Wrapped title and sidebar_label values in quotes to handle special characters (like colons in "Module 3: ...").
- Ensured valid YAML syntax.
- Verified file encoding and whitespace.

## Outcome

- ✅ Impact: Fixed potential Docusaurus build errors due to invalid YAML.
- 🧪 Tests: N/A (Validation via Docusaurus build process).
- 📁 Files: 10 Markdown files updated.
- 🔁 Next prompts: None.
- 🧠 Reflection: Front matter issues are common when generating content; automated cleanup is a good practice.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None
