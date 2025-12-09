---
id: 03-lesson-3
title: "The Digital Mirror"
sidebar_label: "3. Unity Digital Twin"
description: "Visualizing the robot in Unity using ROS-TCP-Connector for high-fidelity rendering."
keywords:
  - unity
  - digital twin
  - ros-tcp
  - visualization
---

# Lesson 3: The Digital Mirror (Unity)

<h2>3.1 Why Unity?</h2>

Gazebo is great for physics, but it looks... functional. **Unity** is an artist's tool. It allows for:
- **Photorealistic rendering**: Shadows, reflections, textures.
- **VR/AR support**: Control the robot in Virtual Reality.
- **Human-Robot Interaction**: Simulating people walking around.

We will use Unity as a "Frontend" for our simulation.

<h2>3.2 Architecture: The TCP Bridge</h2>

We cannot run Unity inside ROS 2. Unity typically runs on Windows or Mac, while ROS runs on Linux. We connect them over the network using **TCP/IP**.

*   **Server**: `ROS-TCP-Endpoint` (Running on ROS 2).
*   **Client**: `ROS-TCP-Connector` (Running inside Unity).

<h2>3.3 Setting Up Unity</h2>

1.  **Install Unity Hub** and download **Unity 2021.3 LTS** (or newer).
2.  Create a new **3D Project**.
3.  Open `Window -> Package Manager`.
4.  Click `+ -> Add package from git URL`.
5.  Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`.

Once installed, you will see a **Robotics** tab in the Unity menu.

<h2>3.4 Scripting the Mirror</h2>

We need a script to tell the Unity object to move when the ROS robot moves.

Create a C# script: `RobotController.cs`.

```csharp title="code/module-2/unity_scripts/RobotController.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// We need the ROS message types. 
// Note: You must generate these using the Robotics -> Generate ROS Messages tool in Unity first.
using RosMessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    // The topic to listen to
    public string topicName = "/cmd_vel";
    
    void Start()
    {
        // Get the ROS Connection instance
        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        
        // Subscribe to the Twist message (Linear/Angular velocity)
        ros.Subscribe<TwistMsg>(topicName, MoveRobot);
    }

    void MoveRobot(TwistMsg msg)
    {
        // 1. Read ROS data (Right-handed coordinate system)
        // ROS: X=Forward, Y=Left, Z=Up
        float linearVelocity = (float)msg.linear.x;
        float angularVelocity = (float)msg.angular.z;

        // 2. Apply to Unity Object (Left-handed coordinate system)
        // Unity: Z=Forward, X=Right, Y=Up
        
        // Move Forward (Z-axis in Unity)
        transform.Translate(Vector3.forward * linearVelocity * Time.deltaTime);
        
        // Rotate (Y-axis in Unity)
        // Note: Negative sign might be needed depending on coordinate mapping
        transform.Rotate(Vector3.up, -angularVelocity * Mathf.Rad2Deg * Time.deltaTime);
    }
}
```

<h2>3.5 Exercise: The Ghost in the Machine</h2>

1.  **ROS 2 Side**: Run the endpoint.
    ```bash
    ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
    ```
2.  **Unity Side**: Press **Play**. You should see "Connected to 127.0.0.1" (or your IP).
3.  **Command**: Open a new terminal and publish a command.
    ```bash
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
    ```

**Result**: Your Unity Cube (representing the robot) starts driving in a circle. You are driving a game object using robotics middleware!