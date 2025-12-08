<!-- Sync Impact Report
Version change: [TEMPLATE] -> 1.0.0
Modified principles: Established Principles I-V based on user input.
Added sections: Technical Stack, Workflow.
Templates requiring updates: None (Initial Setup).
-->
# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Structured Learning
All chapters must follow a uniform, predictable format. Each chapter consists of exactly 3 lessons. Content progression must flow strictly from beginner to intermediate concepts, ensuring a smooth and logical learning curve for the reader.

### II. Clarity & Consistency
Explanations must be clear, simple, and accessible. The tone, terminology, and formatting must remain consistent across all chapters. All code examples provided must be accurate, tested, and runnable to ensure a frictionless user experience.

### III. Technical Fidelity
Robotics content must strictly adhere to official documentation and best practices for ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA. Speculative or deprecated practices are prohibited; the book must reflect the current state of the art.

### IV. RAG Optimization
Content must be written and structured to be cleanly chunkable for Retrieval-Augmented Generation (RAG) systems. The integrated chatbot is constrained to answer questions exclusively from the book's text to ensure accuracy and prevent hallucinations.

### V. Content Integrity
All content must be original and unique to this project. Contradictions across chapters are strictly prohibited; the material must serve as a single, cohesive source of truth for the reader.

## Technical Stack
**Frontend/Publication**: Docusaurus (React-based static site generator).
**RAG System**: Python-based backend for embeddings and retrieval (specific stack TBD per implementation).
**Robotics Simulation**: ROS 2, Gazebo, Unity, NVIDIA Isaac Sim.

## Workflow & Quality Assurance
1. **Drafting**: Content is drafted following the structure principle.
2. **Verification**: Code examples are executed and verified against official docs.
3. **Review**: Content is reviewed for clarity, consistency, and RAG chunkability.
4. **Publishing**: Only content meeting all principles is merged into the main branch.

## Governance
This Constitution supersedes all other project directives.
Amendments to these principles require a documented reasoning and a Semantic Versioning bump.
All contributions (text or code) must be verified against these principles before acceptance.

**Version**: 1.0.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-08