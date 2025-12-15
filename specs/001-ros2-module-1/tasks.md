# Tasks: Module 1 (ROS 2)

**Input**: Design documents from `specs/001-ros2-module-1/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Manual verification via quickstart.md.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create documentation structure docs/module-1/
- [ ] T002 Create code directory structure code/module-1/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [ ] T003 Create Lesson 1 placeholder file docs/module-1/lesson-1-architecture.md
- [ ] T004 Create Lesson 2 placeholder file docs/module-1/lesson-2-control.md
- [ ] T005 Create Lesson 3 placeholder file docs/module-1/lesson-3-urdf.md

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - The First Node (Priority: P1) 🎯 MVP

**Goal**: Student understands ROS 2 computing graph and runs their first node.

**Independent Test**: Run `simple_node.py` and verify log output.

### Implementation for User Story 1

- [X] T006 [P] [US1] Write Lesson 1 Section: Introduction to the Nervous System Metaphor in docs/module-1/lesson-1-architecture.md
- [X] T007 [P] [US1] Write Lesson 1 Section: The Computing Graph (Nodes, Topics, Services) in docs/module-1/lesson-1-architecture.md
- [X] T008 [US1] Implement simple_node.py in code/module-1/simple_node.py
- [X] T009 [US1] Write Lesson 1 Section: Running Your First Node (Referencing simple_node.py) in docs/module-1/lesson-1-architecture.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Controlling the Robot (Priority: P2)

**Goal**: Student understands Pub/Sub communication via `rclpy`.

**Independent Test**: Run `publisher.py` and `subscriber.py` in separate terminals and verify communication.

### Implementation for User Story 2

- [X] T010 [P] [US2] Write Lesson 2 Section: Python Control (rclpy basics) in docs/module-1/lesson-2-control.md
- [X] T011 [US2] Implement publisher.py in code/module-1/publisher.py
- [X] T012 [US2] Implement subscriber.py in code/module-1/subscriber.py
- [X] T013 [US2] Write Lesson 2 Section: The Publisher-Subscriber Pattern (Referencing code examples) in docs/module-1/lesson-2-control.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Defining the Body (Priority: P3)

**Goal**: Student understands URDF structure and visualizes a simple robot.

**Independent Test**: Visualize `simple_humanoid.urdf` using rviz2 or urdf_tutorial.

### Implementation for User Story 3

- [X] T014 [P] [US3] Write Lesson 3 Section: What is URDF? in docs/module-1/lesson-3-urdf.md
- [X] T015 [P] [US3] Write Lesson 3 Section: Humanoid Kinematics (Links & Joints) in docs/module-1/lesson-3-urdf.md
- [X] T016 [US3] Implement simple_humanoid.urdf in code/module-1/simple_humanoid.urdf
- [X] T017 [US3] Write Lesson 3 Section: Visualizing the Body (Referencing simple_humanoid.urdf) in docs/module-1/lesson-3-urdf.md

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T018 [P] Review all content for clarity and tone (Constitution check)
- [X] T019 Verify code examples run on ROS 2 Foxy/Humble (Constitution check)
- [X] T020 Run quickstart.md validation manually

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies
- **Foundational (Phase 2)**: Depends on Setup
- **User Stories (Phase 3+)**: Depend on Foundational
- **Polish (Final Phase)**: Depends on all user stories

### User Story Dependencies

- **User Story 1 (P1)**: Independent
- **User Story 2 (P2)**: Independent (Conceptually builds on US1, but implementation is separate)
- **User Story 3 (P3)**: Independent

### Parallel Opportunities

- All [P] tasks within a phase can run in parallel.
- All User Stories (Phase 3, 4, 5) can theoretically run in parallel after Phase 2, though sequential is recommended for content flow consistency.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1 & 2.
2. Complete Phase 3 (Lesson 1 + Node).
3. Validate `simple_node.py`.

### Incremental Delivery

1. Foundation.
2. US1 (Architecture).
3. US2 (Control).
4. US3 (Body).
