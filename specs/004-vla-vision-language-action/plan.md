# Implementation Plan: Module 4 (VLA)

**Branch**: `004-vla-vision-language-action` | **Date**: 2025-12-08 | **Spec**: [specs/004-vla-vision-language-action/spec.md](spec.md)
**Input**: Feature specification from `specs/004-vla-vision-language-action/spec.md`

## Summary

Implement Module 4 of the "Physical AI & Humanoid Robotics" book. This module introduces the "VLA" (Vision-Language-Action) paradigm, using OpenAI Whisper for voice input and LLMs (GPT-4) for high-level cognitive planning, integrated with ROS 2.

## Technical Context

**Language/Version**: Python 3.8+ (ROS 2 Nodes).
**Frameworks**: 
- **Middleware**: ROS 2 Humble.
- **AI**: OpenAI API (Whisper + GPT), LangChain.
- **Audio**: PyAudio.
**Target Platform**: 
- **Hardware**: x86_64 PC (Internet required for API).
- **OS**: Ubuntu 20.04/22.04 (or WSL2).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] **I. Structured Learning**: 3 lessons (Voice, Planning, Integration).
- [x] **II. Clarity & Consistency**: Python-centric workflow suitable for beginners.
- [x] **III. Technical Fidelity**: Uses industry-standard APIs (OpenAI) and patterns (LangChain).
- [x] **IV. RAG Optimization**: Clear headings and definitions for "VLA", "Grounding".
- [x] **V. Content Integrity**: Original code examples for ROS 2 <-> LLM bridge.

## Project Structure

### Documentation (this feature)

```text
specs/004-vla-vision-language-action/
├── plan.md
├── research.md
├── data-model.md
├── quickstart.md
└── contracts/
```

### Source Code (repository root)

```text
docs/
└── module-4/
    ├── lesson-1-voice.md
    ├── lesson-2-llm-planning.md
    └── lesson-3-capstone.md

code/
└── module-4/
    ├── voice/
    │   └── whisper_node.py
    ├── planning/
    │   └── llm_planner.py
    └── execution/
        └── executive_node.py
```

**Structure Decision**: Separate folders for voice, planning, and execution code to mimic a real microservices architecture.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Cloud APIs | High quality ASR/LLM needed | Local models (Whisper-cpp, Llama 2) require complex setup/hardware. |
| LangChain | Structured output (JSON) is critical | Raw API calls are messy to parse for robot commands. |