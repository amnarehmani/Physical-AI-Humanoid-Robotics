# Quickstart: Module 4 (VLA)

**Purpose**: Verify the Voice-Language-Action pipeline.

## 1. Setup
```bash
export OPENAI_API_KEY="sk-..."
pip install openai langchain pyaudio
```

## 2. Testing Voice
```bash
# Terminal 1
ros2 run code/module-4/voice/whisper_node.py
# Speak into microphone: "Go to the kitchen."
```
*Verification*: `ros2 topic echo /speech/text` shows "Go to the kitchen."

## 3. Testing Planning
```bash
# Terminal 2
ros2 run code/module-4/planning/llm_planner.py
```
*Verification*: `ros2 topic echo /planning/plan` shows `[{"action": "navigate", "target": "kitchen"}]`.

## 4. Testing Execution (Mock)
```bash
# Terminal 3
ros2 run code/module-4/execution/executive_node.py
```
*Verification*: Node logs "Executing: Navigate to Kitchen... Done."
