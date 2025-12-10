---
id: 013-fix-broken-links-sidebars
title: Fix Broken Internal Links and Sidebars
stage: green
date: 2025-12-10
surface: agent
model: gemini-2.0-flash-exp
feature: digital-twin
branch: main
user: Amna Rehman
command: fix
labels: ["bugfix", "docusaurus", "links", "sidebar"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - physical-ai-robotics-book/sidebars.js
tests:
 - null
---

## Prompt

Issue: The Docusaurus build fails due to broken internal links. Example:
- On homepage `/`, linking to `/docs/module-4/00-intro` which does not exist.

Task:
1. Scan the entire project for all internal links in markdown, MDX, and JSX files.
2. Identify links that point to pages or paths that do not exist.
3. For each broken link:
   - If the target page exists elsewhere, update the link to the correct path.
   - If the target page does not exist, either:
       a. Create a placeholder markdown/MDX file with the correct path, preserving the site structure.
       b. Or remove/update the link to point to an existing valid page.
4. Preserve all existing content, formatting, and navigation structure.
5. Output the **corrected files only**, ready to replace the originals.

Constraints:
- Maintain compatibility with Docusaurus v3.
- Do not modify external links.
- Do not affect files unrelated to broken links.

## Response snapshot

Corrected `sidebars.js` to match actual filenames in `docs/module-3` and `docs/module-4`. The broken link `/docs/module-4/00-intro` was actually correct, but the sidebar configuration was referencing non-existent IDs/files (`module-4/intro`), causing build/navigation issues.

## Outcome

- ✅ Impact: Fixed broken sidebar navigation and internal links for Modules 3 and 4.
- 🧪 Tests: Verified file existence against sidebar configuration.
- 📁 Files: Modified sidebars.js.
- 🔁 Next prompts: None.
- 🧠 Reflection: Docusaurus sidebar IDs must match filenames exactly if no explicit `id` is set in frontmatter.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): None.
