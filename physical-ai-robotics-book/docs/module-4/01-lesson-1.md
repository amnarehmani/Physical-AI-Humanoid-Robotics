---
id: lesson-1
title: "The Ear: Voice Control"
sidebar_label: "1. Voice-to-Action"
description: "Using OpenAI Whisper to transcribe voice commands for robot control."
keywords:
  - whisper
  - openai
  - voice
  - asr
  - ros2
---

# Lesson 1: The Ear (Voice Control)

## Introduction

Before a robot can understand *what* we want, it must hear *what* we say. This process is called **Automatic Speech Recognition (ASR)**.

In this lesson, we will build the "Ear" of our robot: a ROS 2 node that listens to the microphone, captures audio buffers, sends them to the OpenAI Whisper API, and publishes the transcribed text to the rest of the system.

## Conceptual Understanding: The Audio Pipeline

Sound is a continuous analog wave. To process it, we must digitize it.

1.  **Sampling**: We measure the air pressure 16,000 times per second (16kHz).
2.  **Buffering**: We collect these samples into chunks (e.g., 1 second).
3.  **Spectrogram**: We convert time-domain data into frequency-domain images.
4.  **Inference**: A Transformer model predicts the sequence of text tokens from the spectrogram.

```text
       (Sound Waves)
             |
             v
+-------------------------+
|   MICROPHONE (HW)       |
|  Analog -> Digital ADC  |
+-------------------------+
             |
      (PCM Bytes 16kHz)
             |
             v
+-------------------------+
|   ROS 2 NODE (Python)   |
| 1. Detect Silence (VAD) |  <-- "Is user speaking?"
| 2. Buffer Audio         |
| 3. API Request (HTTP)   |
+-------------------------+
             |
        (JSON Text)
             |
             v
      /speech/text
```

## System Perspective: OpenAI Whisper

We chose **OpenAI Whisper** for this module. Why?

Traditional ASR (like CMU Sphinx) relied on strict grammar rules. Google Speech API is powerful but often struggles with technical jargon or accents.

Whisper is a **Weakly Supervised Transformer**. It was trained on 680,000 hours of multilingual data from the web. This means it is incredibly robust. It handles background noise, "ums" and "ahs," and robotic terminology (like "Lidar" or "Odometry") significantly better than older models.

## Implementation: The Whisper Node

We will use the `speech_recognition` library as a wrapper around `pyaudio` to handle the low-level microphone buffering.

```python title="code/module-4/voice/whisper_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import openai
import os
import tempfile

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        # Publisher: The rest of the robot listens to this topic
        self.publisher_ = self.create_publisher(String, '/speech/text', 10)
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Calibration: Adjust for ambient noise (e.g., computer fans)
        with self.microphone as source:
            self.get_logger().info("Calibrating microphone... please be silent.")
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
        
        # Ensure API Key is set
        openai.api_key = os.getenv("OPENAI_API_KEY")
        if not openai.api_key:
            self.get_logger().error("OPENAI_API_KEY not found! Please export it.")
            
        self.get_logger().info("Whisper Ear is listening... Speak now.")
        
        # We use a timer to poll the microphone. 
        # In production, this should be a separate thread to avoid blocking ROS.
        self.timer = self.create_timer(0.1, self.listen_loop)

    def listen_loop(self):
        # This implementation uses the synchronous listen() method which blocks.
        # Ideally, we would use listen_in_background(), but this is clearer for learning.
        with self.microphone as source:
            try:
                # listen() waits for speech to start, then waits for it to end (silence)
                # timeout=1.0 means we check for speech every second
                audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=10.0)
                self.transcribe(audio)
            except sr.WaitTimeoutError:
                # No speech detected, just loop again
                pass
            except Exception as e:
                self.get_logger().error(f"Error listening: {e}")

    def transcribe(self, audio_data):
        self.get_logger().info("Transcribing...")
        
        # Whisper requires a file-like object. We use a temp file.
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as temp_wav:
            temp_wav.write(audio_data.get_wav_data())
            temp_filename = temp_wav.name
        
        try:
            with open(temp_filename, "rb") as audio_file:
                # The actual API call to OpenAI
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
            
            text = transcript["text"]
            self.get_logger().info(f"Heard: '{text}'")
            
            # Publish to ROS
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"API Error: {e}")
        finally:
            # Cleanup
            if os.path.exists(temp_filename):
                os.remove(temp_filename)

def main():
    rclpy.init()
    node = WhisperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Engineering Insights: Cloud vs. Edge

We are using the **Cloud API** (`openai.Audio.transcribe`).
*   **Pros**: Extremely accurate, no GPU required on the robot.
*   **Cons**: Latency (1-3 seconds), requires internet, costs money.

For a deployed robot (e.g., a Mars rover), you would use **Whisper.cpp** or **Distil-Whisper** running locally on an NVIDIA Jetson. This reduces latency to &lt;200ms and works offline, but requires careful hardware optimization.

## Exercise: Speak to the Machine

1.  **Hardware Check**: Ensure your microphone is the default input device in Ubuntu Settings.
2.  **Export Key**: `export OPENAI_API_KEY="sk-..."` (Add this to your `~/.bashrc` to make it permanent).
3.  **Run**: `python3 code/module-4/voice/whisper_node.py`
4.  **Verify**: Open a second terminal and watch the topic:
    ```bash
    ros2 topic echo /speech/text
    ```
5.  **Test Phrases**:
    *   "Go to the kitchen." (Simple)
    *   "Navigate to coordinates five point zero and two point five." (Technical)
    *   "Start the patrol sequence." (Abstract)