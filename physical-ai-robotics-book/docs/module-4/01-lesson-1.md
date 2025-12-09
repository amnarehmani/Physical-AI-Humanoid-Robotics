---
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

<h2>1.1 Automatic Speech Recognition (ASR)</h2>

We need to turn sound waves into text strings. **OpenAI Whisper** is a state-of-the-art model for this. It is robust against accents and background noise.

<h2>1.2 Creating the Whisper Node</h2>

We will create a ROS 2 node that listens to the microphone and publishes the text to `/speech/text`.

```python title="code/module-4/voice/whisper_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import openai
import os

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.publisher_ = self.create_publisher(String, '/speech/text', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Ensure API Key is set
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.get_logger().info("Whisper Ear is listening...")
        
        self.timer = self.create_timer(0.1, self.listen_loop)

    def listen_loop(self):
        # In a real loop, we would use a callback or thread
        # This is a simplified synchronous example
        with self.microphone as source:
            try:
                audio = self.recognizer.listen(source, timeout=1.0)
                self.transcribe(audio)
            except sr.WaitTimeoutError:
                pass

    def transcribe(self, audio_data):
        # Save to temp file
        with open("temp.wav", "wb") as f:
            f.write(audio_data.get_wav_data())
        
        # Send to OpenAI
        with open("temp.wav", "rb") as f:
            transcript = openai.Audio.transcribe("whisper-1", f)
        
        text = transcript["text"]
        self.get_logger().info(f"Heard: {text}")
        
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = WhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<h2>1.3 Exercise: Speak</h2>

1.  Export your key: `export OPENAI_API_KEY="sk-..."`
2.  Run the node: `python3 whisper_node.py`
3.  Say "Go to the kitchen".
4.  Check the topic: `ros2 topic echo /speech/text`.