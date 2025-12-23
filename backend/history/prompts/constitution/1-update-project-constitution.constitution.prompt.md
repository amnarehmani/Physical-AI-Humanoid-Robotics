---
id: 1
title: Update Project Constitution
stage: constitution
date: 2025-12-17
surface: agent
model: Qwen
feature: none
branch: main
user: Amna Rehman
command: /sp.constitution
labels: ["constitution", "governance", "principles", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - ".specify/memory/constitution.md"
tests: []
---

# Update Project Constitution

## Prompt Text

```
---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution

Project: Integrated RAG Chatbot Backend for AI/Spec-Driven Book

Purpose:
Design and implement a production-grade Retrieval-Augmented Generation (RAG) backend that powers an embedded chatbot inside a Docusaurus-published book. The chatbot must answer questions grounded strictly in the book’s content, including the ability to answer questions using only user-selected text.

Core principles:
- Groundedness: All responses must be generated exclusively from retrieved book content
- Faithfulness: No hallucinations or unsupported claims
- Traceability: Every response must be traceable to source chunks
- Modularity: Clear separation of ingestion, retrieval, ranking, and generation
- Spec-driven rigor: All behavior derived from explicit specifications
- Provider abstraction: LLM usage via Cohere SDK (no OpenAI dependencies)

Key standards:
- Retrieval:
  - Vector storage: Qdrant Cloud (Free Tier)
  - Embeddings: Cohere embeddings API
  - Chunking strategy must be deterministic and documented
  - Metadata must include source path, section, and chunk ID

- Generation:
  - LLM provider: Cohere (command / chat models)
  - Responses must cite retrieved chunk IDs internally
  - System prompt must explicitly prohibit use of external knowledge

- API:
  - Framework: FastAPI
  - Endpoints:
    - /ingest (book content ingestion)
    - /query (general book QA)
    - /query/selection (answers strictly from user-selected text)
  - All inputs and outputs must be schema-validated

- Storage:
  - Neon Serverless Postgres for:
    - Document metadata
    - Chat sessions
    - Query logs
  - No raw embeddings stored outside Qdrant

- Security & reliability:
  - Rate limiting enabled
  - Input sanitization required
  - Deterministic temperature settings for reproducibility

Constraints:
- No OpenAI SDKs or APIs
- No external web search or browsing
- Responses must fail safely when relevant context is not found
- Selected-text queries must ignore the global vector index entirely
- Codebase must be backend-only (frontend handled by Docusaurus)

Success criteria:
- All chatbot responses are fully grounded in retrieved content
- Selected-text mode answers only from user-provided text
- Zero hallucinated facts during evaluation
- Clear, auditable RAG pipeline
- Backend deployable and documented
- Spec-Kit Plus specifications fully implemented and verifiable

Definition of done:
- End-to-end RAG pipeline operational
- Book content successfully ingested and queried
- Selection-based QA working as specified
- All specs pass validation
- Ready for integration with Docusaurus frontend
```

## Response Text

I have successfully created the project constitution for your Integrated RAG Chatbot Backend for AI/Spec-Driven Book project. The constitution file at `.specify/memory/constitution.md` has been updated with all the specific principles and technical standards you provided.

Key aspects of the created constitution:
- Six core principles reflecting your requirements: Groundedness, Faithfulness, Traceability, Modularity, Spec-driven Rigor, and Provider Abstraction
- Technical standards sections covering retrieval, generation, API, and storage requirements
- Development workflow and quality assurance guidelines
- Governance section establishing the document's authority

The constitution enshrines all the specified constraints including the exclusive use of Cohere SDK, Qdrant Cloud for vector storage, Neon Serverless Postgres for metadata, and FastAPI framework with the specified endpoints.

The document is compliant with all formatting requirements, has no remaining template placeholders, uses proper versioning (1.0.0), and includes a Sync Impact Report as an HTML comment.