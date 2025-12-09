---
id: m2-ch5-fault-injection
title: "Lesson 3: Fault Injection"
sidebar_label: "Lesson 3: Fault Injection"
description: "Simulating failures to test robustness."
keywords:
  - fault injection
  - chaos engineering
  - simulation
  - testing
---

# Lesson 3: Fault Injection

## Breaking Things on Purpose

A Digital Twin is most useful when it tests things that are dangerous or expensive to test in reality.
*   **Motor Failure**: What happens if the right wheel stops?
*   **Sensor Blindness**: What happens if the camera goes black?
*   **Environment Change**: What happens if the lights go out?

## Simulating Motor Failure

We can simulate a dead motor by effectively "unplugging" the bridge command for one wheel, or by sending a `0` velocity command continuously on a high-priority override topic.

Better yet, we can use a Gazebo plugin to detach the joint.
Or, simply set the friction of the wheel to `0` (simulating ice/oil slick).

```bash
# Example: Change friction at runtime via Gazebo Service
gz service -s /world/warehouse/set_parameter ...
```

## Simulating Sensor Noise/Failure

We can modify the sensor noise parameters at runtime or cover the sensor with a visual object.
A simple way to test "Camera Blindness" is to spawn a black box directly in front of the camera link.

## The Chaos Script

Write a Python script `chaos_monkey.py`:
1.  Wait for simulation to start.
2.  Wait 10 seconds.
3.  Spawn a box in front of Bot 1.
4.  Wait 10 seconds.
5.  Teleport Bot 2 to a random location.

Running this alongside your navigation stack tests if your robot can recover from "Kidnapping" and "Dynamic Obstacles."

## End-of-Lesson Checklist

- [ ] I understand the value of "Chaos Engineering" in simulation.
- [ ] I can simulate a sensor failure (e.g., by blocking it).
- [ ] I have written a script to inject a fault into the running simulation.
- [ ] I have observed how my navigation stack reacts to these faults.
