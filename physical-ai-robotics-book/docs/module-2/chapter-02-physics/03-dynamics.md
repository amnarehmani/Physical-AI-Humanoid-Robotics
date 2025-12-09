---
id: m2-ch2-dynamics
title: "Lesson 3: Joint Dynamics"
sidebar_label: "Lesson 3: Joint Dynamics"
description: "Simulating motor resistance and damping."
keywords:
  - urdf
  - joint
  - dynamics
  - damping
  - friction
---

# Lesson 3: Joint Dynamics

## Ideally, Nothing is Ideal

In a perfect world, a joint moves freely with zero resistance. In the real world, joints have:
1.  **Friction**: Static resistance (needs torque to start moving).
2.  **Damping**: Viscous resistance (resists speed). "Thick grease."
3.  **Limits**: Physical stops.

## The Dynamics Tag

Inside a `<joint>` block:

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="arm_link"/>
  <child link="forearm_link"/>
  <axis xyz="0 1 0"/>
  <limit effort="100" velocity="1.0" lower="-1.57" upper="1.57"/>
  
  <dynamics damping="0.1" friction="0.05"/>
</joint>
```

*   `damping`: Force proportional to velocity ($F = -cv$). Helps stabilize controllers.
*   `friction`: Constant force opposing motion ($F = -\mu$).

## Tuning PID Controllers

When you add damping/friction, your PID controllers (which move the joints) might need more power (Integrator term) to reach the goal.
*   **Too much damping**: Robot feels sluggish, lags behind commands.
*   **Too little damping**: Robot oscillates (wobbles) around the target.

## Gravity Compensation

If your arm is heavy, it will fall down when motors are off.
Simulators handle gravity automatically based on the `<mass>` tags we added in Lesson 1.
If your simulated robot sags, your controller needs a "Feed-Forward" term to counteract gravity, just like the real robot.

## End-of-Lesson Checklist

- [ ] I can add `<dynamics>` tags to my URDF joints.
- [ ] I understand the difference between joint friction (constant) and damping (velocity-dependent).
- [ ] I can explain how joint dynamics affect PID tuning.
- [ ] I recognize that gravity effects in simulation come from mass/inertial properties.
