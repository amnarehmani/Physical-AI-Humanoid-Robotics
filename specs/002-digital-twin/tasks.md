# Tasks: Module 2 (Digital Twin)

**Input**: Design documents from `specs/002-digital-twin/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Manual verification via quickstart.md.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create documentation structure docs/module-2/
- [ ] T002 Create code directory structure code/module-2/launch/ code/module-2/urdf/ code/module-2/unity_scripts/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [ ] T003 Create Lesson 1 placeholder file docs/module-2/lesson-1-physics.md
- [ ] T004 Create Lesson 2 placeholder file docs/module-2/lesson-2-sensors.md
- [ ] T005 Create Lesson 3 placeholder file docs/module-2/lesson-3-unity.md

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - The Physics Playground (Priority: P1) 🎯 MVP

**Goal**: Student can spawn a robot in Gazebo and observe physics.

**Independent Test**: Launch `simulation.launch.py` and verify robot spawns and falls.

### Implementation for User Story 1

- [ ] T006 [P] [US1] Write Lesson 1 Section: Introduction to Physics Engines in docs/module-2/lesson-1-physics.md
- [ ] T007 [US1] Create simulation.launch.py in code/module-2/launch/simulation.launch.py (Basic spawn only)
- [ ] T008 [US1] Write Lesson 1 Section: Setting up the World (Gravity, Ground Plane) in docs/module-2/lesson-1-physics.md
- [ ] T009 [US1] Write Lesson 1 Section: Launching the Simulation (Referencing launch file) in docs/module-2/lesson-1-physics.md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Seeing the World (Priority: P2)

**Goal**: Student can simulate and visualize sensor data (LiDAR, Camera, IMU).

**Independent Test**: Run `simulation.launch.py` (with updated Xacro) and verify `/scan`, `/camera/image_raw`, `/imu/data` topics.

### Implementation for User Story 2

- [ ] T010 [P] [US2] Write Lesson 2 Section: Principles of Perception (LiDAR, Camera, IMU) in docs/module-2/lesson-2-sensors.md
- [ ] T011 [US2] Create sensors.xacro in code/module-2/urdf/sensors.xacro (Gazebo plugins)
- [ ] T012 [US2] Update simulation.launch.py to include sensors.xacro and bridge configuration in code/module-2/launch/simulation.launch.py
- [ ] T013 [US2] Write Lesson 2 Section: Simulating Sensors with Plugins in docs/module-2/lesson-2-sensors.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - The Digital Mirror (Priority: P3)

**Goal**: Student can visualize the robot in Unity synchronized with ROS 2.

**Independent Test**: Run Unity Scene + Gazebo and verify movement synchronization.

### Implementation for User Story 3

- [ ] T014 [P] [US3] Write Lesson 3 Section: The Digital Twin Concept in docs/module-2/lesson-3-unity.md
- [ ] T015 [US3] Create RobotController.cs in code/module-2/unity_scripts/RobotController.cs
- [ ] T016 [US3] Write Lesson 3 Section: Setting up Unity Robotics Hub in docs/module-2/lesson-3-unity.md
- [ ] T017 [US3] Write Lesson 3 Section: Creating the Bridge (ROS-TCP-Connector) in docs/module-2/lesson-3-unity.md

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T018 [P] Review all content for clarity and tone (Constitution check)
- [ ] T019 Verify code examples run on ROS 2 Humble + Gazebo Fortress + Unity 2021.3+
- [ ] T020 Run quickstart.md validation manually

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies
- **Foundational (Phase 2)**: Depends on Setup
- **User Stories (Phase 3+)**: Depend on Foundational
- **Polish (Final Phase)**: Depends on all user stories

### User Story Dependencies

- **User Story 1 (P1)**: Independent
- **User Story 2 (P2)**: Extends US1 code (sensors.xacro is included in launch file), but content is separate.
- **User Story 3 (P3)**: Depends on US2 infrastructure (needs ROS 2 running to bridge).

### Parallel Opportunities

- All [P] tasks within a phase can run in parallel.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1 & 2.
2. Complete Phase 3 (Lesson 1 + Launch file).
3. Validate Gazebo spawn.

### Incremental Delivery

1. Foundation.
2. US1 (Physics).
3. US2 (Sensors).
4. US3 (Unity).
