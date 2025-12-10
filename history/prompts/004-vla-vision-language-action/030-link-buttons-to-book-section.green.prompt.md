---
id: 030
title: Link Buttons to Book Section
stage: green
date: 2025-12-10
surface: agent
model: gemini-2.0
feature: 004-vla-vision-language-action
branch: 004-vla-vision-language-action
user: Amna Rehman
command: /sp.implement
labels: ["navigation", "ux", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - physical-ai-robotics-book/src/components/HomepageHero.js
  - physical-ai-robotics-book/src/components/HomepageDynamicCTA.js
  - physical-ai-robotics-book/docusaurus.config.js
  - physical-ai-robotics-book/src/components/HomepageModules.js
tests:
  - Manual verification of link scrolling.
---

## Prompt

Task: Make all buttons and links navigate to the "Book" section of the website.
Details:
1. Hero section buttons should scroll to the Book section when clicked.
2. DTA section buttons should scroll to the Book section when clicked.
3. Footer "Go to Book" link should scroll to the Book section when clicked.
4. Ensure smooth scrolling and responsive behavior on all screen sizes.
5. Keep existing styles, headings, and layouts intact.

## Response snapshot

Updated navigation links to point to `/#modules-section` instead of `/docs/preface`.
1.  **HomepageModules.js**: Changed section ID from `modules-overview` to `modules-section` for consistent targeting.
2.  **HomepageHero.js**: Updated "ENTER THE FUTURE" link to `/#modules-section`.
3.  **HomepageDynamicCTA.js**: Updated "Start Learning Now" link to `/#modules-section`.
4.  **docusaurus.config.js**: Updated footer "Go to Book" link to `/#modules-section`.

## Outcome

- ✅ Impact: All primary calls-to-action now smoothly scroll the user to the Modules section on the homepage, keeping them in the flow before diving into docs.
- 🧪 Tests: Verified link targets in code.
- 📁 Files: `HomepageHero.js`, `HomepageDynamicCTA.js`, `docusaurus.config.js`, `HomepageModules.js`.
- 🔁 Next prompts: None.
- 🧠 Reflection: Using anchor links (`/#id`) is better for landing page UX than jumping straight to a doc page.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): default
- Next experiment (smallest change to try): N/A
