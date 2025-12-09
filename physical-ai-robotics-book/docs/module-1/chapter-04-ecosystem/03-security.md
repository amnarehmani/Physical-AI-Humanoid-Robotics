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
---

# Lesson 3: SROS2 & Security

## The Insecure Default

By default, ROS 2 trusts everyone on the network. Any laptop connected to your WiFi can publish `/cmd_vel` and drive your robot into a wall. This is unacceptable for products.

## SROS2 (Secure ROS 2)

ROS 2 uses **DDS Security** standard, which provides:
1.  **Authentication**: "Are you who you say you are?" (Certificates).
2.  **Access Control**: "Are you allowed to publish this topic?" (Permissions).
3.  **Encryption**: "Can anyone else read this data?" (AES-GCM).

## Enabling Security

1.  **Create a Keystore**:
    ```bash
    ros2 security create_keystore my_keystore
    ```
2.  **Create Keys for a Node**:
    ```bash
    ros2 security create_key my_keystore /my_talker_node
    ```
3.  **Run with Security**:
    ```bash
    export ROS_SECURITY_KEYSTORE=~/my_keystore
    export ROS_SECURITY_ENABLE=true
    export ROS_SECURITY_STRATEGY=Enforce
    
    ros2 run demo_nodes_cpp talker --ros-args --enclave /my_talker_node
    ```

If you try to run a listener without a valid key, it will receive nothing. The data is encrypted on the wire.

## Limitations

*   **Performance**: Encryption adds latency and CPU load.
*   **Complexity**: Managing certificates for a fleet of 100 robots is hard.
*   **Discovery**: Secured discovery can be slower.

## End-of-Lesson Checklist

- [ ] I understand the three pillars of SROS2 (Auth, Access, Encryption).
- [ ] I have generated a keystore (even if just for testing).
- [ ] I know how to set the environment variables to enforce security.
- [ ] I understand the trade-off between security and performance.
