---
id: m4-ch4-pipeline
title: "Lesson 1: The VLA Pipeline"
sidebar_label: "Lesson 1: Pipeline"
description: "Connecting Vision, Language, and Action nodes."
keywords:
  - pipeline
  - architecture
  - ros2
  - json
---

# Lesson 1: The VLA Pipeline

## Step 1: Perception (Vision to Text)

We use a "Open Vocabulary Detector" (like OWL-ViT or Grounding DINO).
Input: Image.
Prompts: "trash", "tool", "fruit".
Output: Bounding Boxes + Labels.

```python
# perception_node.py
def image_callback(self, msg):
    image = bridge.imgmsg_to_cv2(msg)
    detections = detector.detect(image, ["trash", "tool", "fruit"])
    # detections = [{"label": "fruit", "bbox": [10, 10, 50, 50], "center": [30, 30]}]
    
    msg = String()
    msg.data = json.dumps(detections)
    self.pub.publish(msg)
```

## Step 2: Reasoning (Text to Text)

The Brain Node listens.

```python
# brain_node.py
def detection_callback(self, msg):
    objects = json.loads(msg.data)
    prompt = f"I see {objects}. The user wants to tidy up. Trash goes to bin. Tools go to box. Fruit goes to bowl. Generate a plan."
    
    plan = llm.generate(prompt)
    # Plan: [{"action": "pick", "target": "fruit", "dest": "bowl"}]
    
    self.pub_plan.publish(json.dumps(plan))
```

## Step 3: Action (Text to Motion)

The Control Node executes.

```python
# control_node.py
def plan_callback(self, msg):
    plan = json.loads(msg.data)
    for step in plan:
        target_obj = find_object_by_label(step['target'])
        robot.pick(target_obj['center'])
        robot.place(locations[step['dest']])
```

## End-of-Lesson Checklist

- [ ] I have established the ROS 2 topic flow: `/detections` -> `/plan`.
- [ ] I define the interface format (JSON) between nodes.
- [ ] I understand how the visual label ("fruit") is passed to the LLM.
- [ ] I understand how the LLM's decision ("bowl") is mapped to a coordinate by the Control Node.
