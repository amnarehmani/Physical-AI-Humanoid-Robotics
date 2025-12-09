# Tasks: Module 3 (NVIDIA Isaac)

**Input**: Design documents from `specs/003-isaac-sim-brain/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Manual verification via quickstart.md.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create documentation structure docs/module-3/
- [ ] T002 Create code directory structure code/module-3/isaac_sim/ code/module-3/isaac_ros/ code/module-3/nav2/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [ ] T003 Create Lesson 1 placeholder file docs/module-3/lesson-1-isaac-sim.md
- [ ] T004 Create Lesson 2 placeholder file docs/module-3/lesson-2-isaac-ros.md
- [ ] T005 Create Lesson 3 placeholder file docs/module-3/lesson-3-nav2.md

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Photorealistic Simulation (Priority: P1) 🎯 MVP

**Goal**: Student can load a robot into Isaac Sim and set up a scene.

**Independent Test**: Run `load_humanoid.py` in Isaac Sim Script Editor.

### Implementation for User Story 1

- [ ] T006 [P] [US1] Write Lesson 1 Section: Introduction to Isaac Sim & USD in docs/module-3/lesson-1-isaac-sim.md
- [ ] T007 [US1] Create load_humanoid.py in code/module-3/isaac_sim/load_humanoid.py
- [ ] T008 [US1] Write Lesson 1 Section: Setting up the Environment (Nucleus Assets) in docs/module-3/lesson-1-isaac-sim.md
- [ ] T009 [US1] Write Lesson 1 Section: Automating Scene Loading (Referencing script) in docs/module-3/lesson-1-isaac-sim.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Accelerated Perception (Priority: P2)

**Goal**: Student can run Visual SLAM using Isaac ROS.

**Independent Test**: Launch `visual_slam.launch.py` and visualize `odom` in Rviz.

### Implementation for User Story 2

- [ ] T010 [P] [US2] Write Lesson 2 Section: Hardware Acceleration & Isaac ROS in docs/module-3/lesson-2-isaac-ros.md
- [ ] T011 [US2] Create visual_slam.launch.py in code/module-3/isaac_ros/visual_slam.launch.py
- [ ] T012 [US2] Write Lesson 2 Section: Running Visual SLAM (Docker workflow) in docs/module-3/lesson-2-isaac-ros.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Navigation & Planning (Priority: P3)

**Goal**: Student can navigate the robot autonomously using Nav2.

**Independent Test**: Launch Nav2 with `nav2_humanoid_params.yaml` and set a goal.

### Implementation for User Story 3

- [ ] T013 [P] [US3] Write Lesson 3 Section: The Navigation Stack (Nav2) in docs/module-3/lesson-3-nav2.md
- [ ] T014 [US3] Create nav2_humanoid_params.yaml in code/module-3/nav2/nav2_humanoid_params.yaml
- [ ] T015 [US3] Write Lesson 3 Section: Configuring Costmaps for Humanoids in docs/module-3/lesson-3-nav2.md
- [ ] T016 [US3] Write Lesson 3 Section: Launching Navigation (Referencing config) in docs/module-3/lesson-3-nav2.md

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T017 [P] Review all content for clarity and tone (Constitution check)
- [ ] T018 Verify code examples logic (Note: Execution requires Nvidia hardware)
- [ ] T019 Run quickstart.md validation manually

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies
- **Foundational (Phase 2)**: Depends on Setup
- **User Stories (Phase 3+)**: Depend on Foundational
- **Polish (Final Phase)**: Depends on all user stories

### User Story Dependencies

- **User Story 1 (P1)**: Independent
- **User Story 2 (P2)**: Independent (Conceptually builds on Sim, but implementation is separate launch file).
- **User Story 3 (P3)**: Independent (Config file creation).

### Parallel Opportunities

- All [P] tasks within a phase can run in parallel.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1 & 2.
2. Complete Phase 3 (Lesson 1 + Script).
3. Validate Script.

### Incremental Delivery

1. Foundation.
2. US1 (Sim).
3. US2 (Perception).
4. US3 (Nav).
