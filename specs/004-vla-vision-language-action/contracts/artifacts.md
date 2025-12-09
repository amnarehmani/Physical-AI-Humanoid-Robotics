# Code Artifacts Contract: Module 4

## 1. Voice Artifacts

### `whisper_node.py`
- **Purpose**: Captures audio and publishes transcribed text.
- **Inputs**: Microphone Audio (PyAudio).
- **Outputs**: Topic `/speech/text` (`std_msgs/String`).
- **Config**: `OPENAI_API_KEY` (env var).

## 2. Planning Artifacts

### `llm_planner.py`
- **Purpose**: Translates natural language into structured plans.
- **Inputs**: 
    - Topic `/speech/text` (`std_msgs/String`).
    - Topic `/vision/detections` (`vision_msgs/Detection2DArray` - *Mocked for now*).
- **Outputs**: Topic `/planning/plan` (`std_msgs/String` - JSON Array).
- **Logic**:
    1. Receive text "Get the apple".
    2. Query LLM with system prompt + available objects.
    3. LLM returns `[{"action": "navigate", "target": "kitchen"}, {"action": "pick", "object": "apple"}]`.
    4. Publish JSON string.

## 3. Execution Artifacts

### `executive_node.py`
- **Purpose**: Parses the plan and calls ROS 2 actions.
- **Inputs**: Topic `/planning/plan` (JSON String).
- **Outputs**: 
    - Action Client `navigate_to_pose` (Nav2).
    - Action Client `pickup_object` (Mock Manipulation).
