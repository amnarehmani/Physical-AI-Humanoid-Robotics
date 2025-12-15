# Tasks: Module 4 (VLA)

**Input**: Design documents from `specs/004-vla-vision-language-action/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Manual verification via quickstart.md.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create documentation structure docs/module-4/
- [ ] T002 Create code directory structure code/module-4/voice/ code/module-4/planning/ code/module-4/execution/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [ ] T003 Create Lesson 1 placeholder file docs/module-4/lesson-1-voice.md
- [ ] T004 Create Lesson 2 placeholder file docs/module-4/lesson-2-llm-planning.md
- [ ] T005 Create Lesson 3 placeholder file docs/module-4/lesson-3-capstone.md

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Voice Control (Priority: P1) 🎯 MVP

**Goal**: Student can use voice commands to publish text to ROS 2.

**Independent Test**: Run `whisper_node.py`, speak, and verify text on `/speech/text`.

### Implementation for User Story 1

- [x] T006 [P] [US1] Write Lesson 1 Section: Intro to ASR and OpenAI Whisper in docs/module-4/lesson-1-voice.md
- [ ] T007 [US1] Create whisper_node.py in code/module-4/voice/whisper_node.py
- [x] T008 [US1] Write Lesson 1 Section: Creating the Ear Node (Code walkthrough) in docs/module-4/lesson-1-voice.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Cognitive Planning (Priority: P2)

**Goal**: Student can translate text commands into structured plans using an LLM.

**Independent Test**: Run `llm_planner.py`, publish text, verify JSON plan on `/planning/plan`.

### Implementation for User Story 2

- [x] T009 [P] [US2] Write Lesson 2 Section: The Cognitive Loop & LLMs in docs/module-4/lesson-2-llm-planning.md
- [ ] T010 [US2] Create llm_planner.py in code/module-4/planning/llm_planner.py
- [x] T011 [US2] Write Lesson 2 Section: Prompt Engineering for Robotics in docs/module-4/lesson-2-llm-planning.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - The Capstone Walkthrough (Priority: P3)

**Goal**: Student understands how to integrate Voice, Planning, and Action (Simulated).

**Independent Test**: Run `executive_node.py` with a sample plan and verify log output.

### Implementation for User Story 3

- [x] T012 [P] [US3] Write Lesson 3 Section: System Architecture (Diagrams) in docs/module-4/lesson-3-capstone.md
- [ ] T013 [US3] Create executive_node.py in code/module-4/execution/executive_node.py (Mock execution)
- [x] T014 [US3] Write Lesson 3 Section: The Executive Node (State Machine) in docs/module-4/lesson-3-capstone.md
- [x] T015 [US3] Write Lesson 3 Section: Putting it All Together (Launch instructions) in docs/module-4/lesson-3-capstone.md

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T016 [P] Review all content for clarity and tone (Constitution check)
- [ ] T017 Verify code examples run (requires OpenAI API Key)
- [ ] T018 Run quickstart.md validation manually

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies
- **Foundational (Phase 2)**: Depends on Setup
- **User Stories (Phase 3+)**: Depend on Foundational
- **Polish (Final Phase)**: Depends on all user stories

### User Story Dependencies

- **User Story 1 (P1)**: Independent
- **User Story 2 (P2)**: Independent (Can mock input text).
- **User Story 3 (P3)**: Independent (Can mock input plan).

### Parallel Opportunities

- All [P] tasks within a phase can run in parallel.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1 & 2.
2. Complete Phase 3 (Lesson 1 + Whisper Node).
3. Validate Voice Input.

### Incremental Delivery

1. Foundation.
2. US1 (Voice).
3. US2 (Planning).
4. US3 (Execution).
