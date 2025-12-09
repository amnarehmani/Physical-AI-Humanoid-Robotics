# Data Model: Module 4 Content Structure

## 1. Hierarchy

### Module: Vision-Language-Action (VLA)
- **ID**: `module-4`
- **Slug**: `/modules/vla`
- **Metadata**:
    - `title`: "Vision-Language-Action (VLA)"
    - `difficulty`: "Advanced"
    - `prerequisites`: ["Module 3 (Isaac Brain)", "OpenAI API Key"]

## 2. Lesson Definitions

### Lesson 1: Voice-to-Action
- **Filename**: `lesson-1-voice.md`
- **Title**: "The Ear: Voice Control with Whisper"
- **Key Concepts**: Audio Capture, ASR (Automatic Speech Recognition), ROS 2 String Msgs.
- **Artifacts**: `whisper_node.py`.

### Lesson 2: The Cognitive Brain
- **Filename**: `lesson-2-llm-planning.md`
- **Title**: "The Brain: Reasoning with LLMs"
- **Key Concepts**: LangChain, Prompt Engineering, JSON Parsing, Action Dispatching.
- **Artifacts**: `llm_planner.py`.

### Lesson 3: The Capstone
- **Filename**: `lesson-3-capstone.md`
- **Title**: "Capstone: The Butler Robot"
- **Key Concepts**: Integration, State Machines, Full Loop (Voice -> Action).
- **Artifacts**: `capstone_launch.py`, `system_architecture.mermaid`.

## 3. Entity Relationships
- **Microphone** --[Audio]--> **Whisper Node**
- **Whisper Node** --[/speech/text]--> **LLM Planner**
- **Vision System** --[/vision/objects]--> **LLM Planner**
- **LLM Planner** --[/planning/plan]--> **Executive Node**
- **Executive Node** --[Action Clients]--> **Nav2 / MoveIt**
