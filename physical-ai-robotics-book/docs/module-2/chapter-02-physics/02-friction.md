---
id: m2-ch2-friction
title: "Lesson 2: Friction and Contact"
sidebar_label: "Lesson 2: Friction"
description: "Tuning surface properties for realistic interaction."
keywords:
  - gazebo
  - friction
  - mu1
  - mu2
  - kp
  - kd
---

# Lesson 2: Friction and Contact

## 1. Introduction

In a perfect mathematical world, a wheel turns and the robot moves forward. In the real world, the wheel might spin on ice, the robot might bounce on a rubber floor, or sink into mud.

**Friction** and **Contact Dynamics** are the parameters that define how our robot interacts with the environment. If you get these wrong, your walking robot will slip like Bambi on ice, or your gripper will drop every object it touches.

In this lesson, we will explore the `<gazebo>` tags used to tune these physical properties.

## 2. Conceptual Understanding: Coulomb Friction

Gazebo primarily uses the **Coulomb Friction Model**.

$$ F_f \leq \mu F_N $$

*   $F_f$: Friction Force.
*   $F_N$: Normal Force (pushing surfaces together).
*   $\mu$: Coefficient of Friction.

In Gazebo, we define two coefficients:
1.  **$\\mu_1$**: Friction coefficient in the primary direction (e.g., forward for a wheel).
2.  **$\\mu_2$**: Friction coefficient in the secondary direction (e.g., sideways for a wheel).

### The Friction Cone Visualization

```text
       Normal Force (Fn)
             ^ 
             | 
             | 
      \      |      / 
       \     |     /   <- Friction Cone (Angle depends on Mu)
        \    |    / 
         \   |   / 
    -------------------  Surface
```
If the force vector stays inside the cone, the object sticks. If it goes outside, it slips.

## 3. System Perspective: The Contact Model

When two objects touch in simulation, they don't just "stop." They compress slightly. Gazebo models contact as a **Spring-Damper System**.

```text
      Robot Foot
         | 
    [ Spring (Kp) ]   <-- Resists penetration (Hardness)
    [ Damper (Kd) ]   <-- Resists velocity (Bounciness)
         | 
    ----------------  Ground
```

*   **$k_p$ (Stiffness)**: The spring constant.
    *   **High ($10^{12}$)**: Hard (Steel).
    *   **Low ($10^4$)**: Soft (Sponge, Rubber).
*   **$k_d$ (Damping)**: The damper constant.
    *   **High (100)**: Absorb energy (Mud, Dough).
    *   **Low (1)**: Bouncy (Superball).

## 4. Implementation: Configuring Friction in URDF

Standard URDF doesn't have tags for friction. We must use the `<gazebo>` extension tag, referencing the link name.

```xml
<robot name="my_robot">
  
  <!-- The Wheel Link -->
  <link name="left_wheel">
    <collision>
      <geometry><cylinder radius="0.1" length="0.05"/></geometry>
    </collision>
    <inertial> ... </inertial>
  </link>

  <!-- Physics Properties -->
  <gazebo reference="left_wheel">
    <!-- Friction Coefficients -->
    <mu1>1.0</mu1>  <!-- Forward Grip -->
    <mu2>1.0</mu2>  <!-- Sideways Grip -->
    
    <!-- Contact Stiffness (Optional, defaults are usually fine) -->
    <kp>1000000.0</kp> 
    <kd>1.0</kd>
    
    <!-- Material Color (Visual only, but useful) -->
    <material>Gazebo/Black</material>
  </gazebo>

</robot>
```

## 5. Real-World Example: The Omni-Wheel

Imagine a **Mecanum Wheel** or **Omni-Wheel**. It has rollers that allow it to slide sideways.
How do we simulate this?
We cheat using friction directions!

*   Set $\\mu_1 = 1.0$ (High grip forward).
*   Set $\\mu_2 = 0.0$ (Zero grip sideways).

Now, if you apply force sideways, the wheel slides perfectly. This is much computationally cheaper than simulating 12 tiny individual rollers.

## 6. Engineering Insights: The "Jittering" Robot

**Symptom**: Your robot is standing still on the ground, but it is vibrating or slowly drifting.
**Cause**: The contact stiffness ($k_p$) is too high for the solver time-step. The physics engine detects penetration, applies a huge "push back" force (spring), overshoots, detects separation, applies gravity, overshoots again.
**Fix**:
1.  **Reduce $k_p$**: Make the ground slightly softer.
2.  **Increase $k_d$**: Add damping to kill the oscillation energy.
3.  **Decrease Simulation Step**: Make the solver run faster (e.g., 0.001s $\to$ 0.0001s).

## 7. Summary

*   **$\\mu$ (Mu)** controls "slip vs grip." High for tires/feet, low for sliding joints.
*   **$k_p/k_d$** control "hardness vs bounciness."
*   **Collision Geometry** must be simple (Cylinders/Boxes) for these calculations to be fast and stable.

In the final lesson of this chapter, we will look at **Joint Dynamics**—modeling the friction inside the motors themselves.
