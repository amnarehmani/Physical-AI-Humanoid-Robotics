---
id: m2-ch3-imu
title: "Lesson 3: IMU and Noise"
sidebar_label: "Lesson 3: IMU & Noise"
description: "Simulating accelerometers, gyros, and sensor noise."
keywords:
  - gazebo
  - imu
  - noise
  - gaussian
---

# Lesson 3: IMU and Noise

## The Inner Ear

The Inertial Measurement Unit (IMU) provides acceleration and rotational velocity. It is essential for balancing humanoids and estimating robot orientation (odometry).

```xml
<gazebo reference="imu_link">
  <gravity>true</gravity>
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <topicName>imu</topicName>
      <bodyName>imu_link</bodyName>
      <updateRateHZ>10.0</updateRateHZ>
      <gaussianNoise>0.0</gaussianNoise>
      <xyzOffset>0 0 0</xyzOffset>
      <rpyOffset>0 0 0</rpyOffset>
      <frameName>imu_link</frameName>
      <initialOrientationAsReference>false</initialOrientationAsReference>
    </plugin>
  </sensor>
</gazebo>
```

## The Reality of Noise

Real sensors are noisy. A stationary robot's Lidar might report 1.00m, 1.01m, 0.99m.
If you simulate with **Perfect Sensors**, your algorithms will be "spoiled." They will work in sim but fail in reality.

### Adding Noise

Gazebo allows you to inject Gaussian noise into sensors.

```xml
<ray>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.01</stddev> <!-- 1cm error -->
  </noise>
</ray>
```

**Best Practice**: Always add a small amount of noise to your simulation sensors. It forces you to write robust code (e.g., using Kalman Filters) rather than relying on perfect data.

## End-of-Lesson Checklist

- [ ] I can add an IMU sensor to my robot.
- [ ] I understand the importance of `update_rate` (IMUs usually run fast, >100Hz).
- [ ] I can inject Gaussian noise into my sensor definitions.
- [ ] I understand why "Perfect Sensors" are bad for algorithm development.
