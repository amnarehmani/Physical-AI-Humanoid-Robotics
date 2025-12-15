---
id: m1-ch4-security
title: "Lesson 3: SROS2 & Security"
sidebar_label: "Lesson 3: Security"
description: "Securing ROS 2 traffic with DDS Security."
keywords:
  - ros2
  - security
  - sros2
  - dds
  - encryption
  - authentication
---

# Lesson 3: SROS2 & Security

## 1. Introduction

By default, ROS 2 is promiscuous. It trusts everyone.
If you connect your laptop to the same WiFi as a hospital robot, you can subscribe to its camera stream or publish `cmd_vel` to drive it into a wall.

This was fine for research labs in 2010. It is unacceptable for commercial products in 2025.
This lesson introduces **SROS2** (Secure ROS 2), the set of tools that allows you to lock down your robot's nervous system.

## 2. Conceptual Understanding: The Immune System

**Intuition**:
*   **The Virus**: A hacker on the network injecting fake data.
*   **The Antibody**: SROS2.
*   **The Mechanism**:
    1.  **ID Card (Authentication)**: "Who are you?" Every node must present a valid x.509 certificate signed by a Certificate Authority (CA).
    2.  **Keycard (Access Control)**: "Where can you go?" Even if you are a valid node, are you allowed to publish `/cmd_vel`? Or just listen to `/battery`?
    3.  **Envelope (Encryption)**: "Can anyone read your mail?" The data payload is encrypted (AES-GCM) so eavesdroppers see noise.

## 3. System Perspective: The Secure Handshake

How does a secure connection happen? It occurs deep in the middleware (DDS).

```mermaid-text
[Node A: Talker]                      [Node B: Listener]
       |                                      |
       | (1) Discovery Announcement           |
       |     "I am Node A. Here is my Cert."  |
       |------------------------------------->|
       |                                      |
       | (2) Verify Cert against CA           |
       |     "Cert is valid."                 |
       |                                      |
       | (3) Diffie-Hellman Key Exchange      |
       |<------------------------------------>|
       |                                      |
       | (4) Encrypted Data Stream            |
       |     [ #$*(@#&($*@#... ]              |
       |------------------------------------->|
```

If Node B does not have a valid certificate, the handshake fails at step 2. Node A never sends the encryption keys.

## 4. Practical Example: Locking the Door

We will generate security artifacts for a simple Talker/Listener demo.

### Step 1: The Keystore
The Keystore is the vault that holds the Certificate Authority (CA).
```bash
ros2 security create_keystore demo_keystore
```

### Step 2: The Keys
We create a private key and certificate for our node.
```bash
# syntax: create_key <keystore_path> <node_name>
ros2 security create_key demo_keystore /talker_node
ros2 security create_key demo_keystore /listener_node
```

### Step 3: The Enforced Run
We tell ROS 2 to **Force** security. If keys are missing, the node will refuse to start.
```bash
export ROS_SECURITY_KEYSTORE=$(pwd)/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Terminal 1
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_node

# Terminal 2 (Without keys -> FAILS)
ros2 run demo_nodes_cpp listener

# Terminal 3 (With keys -> WORKS)
ros2 run demo_nodes_cpp listener --ros-args --enclave /listener_node
```

## 5. Engineering Insights: The Cost of Safety

Security is not free.

*   **Latency**: The handshake takes time (seconds). Once connected, symmetric encryption (AES) adds microseconds to each message. For 100Hz control loops, this is usually negligible. For 4K video, it can burn CPU.
*   **Discovery**: Secure discovery is multicast-heavy and slower. Large fleets might need a **Discovery Server**.
*   **Provisioning**: Managing keys for 1,000 robots is a DevOps nightmare. You need a pipeline to rotate expired keys automatically.

## 6. Summary

SROS2 turns ROS 2 from a research toy into an industrial-grade platform.
1.  **Authentication** keeps intruders out.
2.  **Encryption** keeps secrets safe.
3.  **Access Control** limits the blast radius of a compromised node.

You now have a complete picture of the ROS 2 Ecosystem: **Recording (Bag)**, **Diagnosing (Doctor)**, and **Securing (SROS2)**. This concludes Chapter 4.