# Research: Module 4 (Vision-Language-Action)

**Status**: Complete
**Date**: 2025-12-08

## 1. Technical Decisions

### Voice-to-Action Pipeline
- **Component**: **OpenAI Whisper** (API or `faster-whisper` local).
- **Reasoning**: 
    - Industry standard for robustness.
    - Python API is simple for beginners (`import openai`).
    - Local option (`faster-whisper`) allows offline use if GPU available.
- **Integration**: Python node `whisper_node.py` that captures audio via `PyAudio` and publishes to `/speech/text` (String).

### Cognitive Planning (LLM)
- **Component**: **OpenAI GPT-4o** (or GPT-3.5 Turbo) via **LangChain**.
- **Reasoning**:
    - Direct JSON output mode is reliable for robot commands.
    - LangChain provides a structure for "Tools" (ROS 2 actions).
- **Integration**: Python node `planner_node.py` that subscribes to `/speech/text` and outputs a sequence of actions to `/planning/plan`.

### VLA Architecture
- **Definition**: We are building a *modular* VLA system (Vision + Language -> Planner -> Action), not training an end-to-end transformer (like RT-2) which is too heavy for this book.
- **Flow**: 
    1. **Vision**: Object detection (YOLO/Isaac ROS) provides "World State".
    2. **Language**: User command + World State -> LLM.
    3. **Action**: LLM outputs function calls (`navigate_to("kitchen")`, `pick("apple")`).

## 2. Key Concepts to Cover

### Cognitive Architecture
- **The OODA Loop**: Observe (Vision), Orient (State Update), Decide (LLM Plan), Act (ROS 2 Action).
- **Grounding**: Ensuring the LLM doesn't hallucinate objects that don't exist. We "ground" the prompt with a list of detected objects from the Vision module.

### Prompt Engineering for Robotics
- **System Prompt**: "You are a robot planner. Output valid JSON only. Available actions: [move, pick, place]."
- **Chain of Thought**: Asking the LLM to "Think step-by-step" before outputting the JSON plan improves safety.

## 3. Artifacts & References
- **OpenAI API Docs**: https://platform.openai.com/docs/
- **LangChain**: https://python.langchain.com/
- **Whisper**: https://github.com/openai/whisper
