<!-- Sync Impact Report
Version change: 1.0.0 -> 1.1.0
Modified principles: Renamed and expanded "Structured Learning" to "Immutable Structure" and "Deep Educational Content". Refined "Clarity & Consistency" into "Professional Style".
Added sections: "Visual Explanation", "Standardized Output".
Templates requiring updates: None (Rules apply to content generation).
-->
# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Immutable Structure
The project's high-level architecture is fixed. **DO NOT** add or remove modules, chapters, or lessons. **DO NOT** change titles or reorder existing units. All development work must focus exclusively on expanding the content within the pre-existing lesson files.

### II. Deep Educational Content
Lessons must be comprehensive and detailed. Content flow must strictly follow this sequence: **Intuition → Theory → System → Example → Limitations**. Explanations must be grounded in real humanoid robot scenarios, explicitly addressing failure cases, trade-offs, and system limitations. Shallow summaries are prohibited.

### III. Visual Explanation
Every lesson must include at least **one text-based diagram** (ASCII art or step-flow) to visualize complex concepts. All components within these diagrams must be clearly labeled to aid understanding.

### IV. Professional Style
The writing style must be strictly educational and objective. Marketing tone, fluff, and sections consisting solely of bullet points are prohibited. Use clear headings and subheadings to organize deep technical content.

### V. Standardized Output
All generated content must be **Docusaurus-compatible Markdown**. This requires correct frontmatter, properly formatted code blocks (including those for diagrams), and the use of Docusaurus-specific features like admonitions (callouts) where useful.

### VI. Technical Fidelity
Robotics content must strictly adhere to official documentation and best practices for ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA. Speculative or deprecated practices are prohibited; the book must reflect the current state of the art.

### VII. RAG Optimization
Content must be written and structured to be cleanly chunkable for Retrieval-Augmented Generation (RAG) systems. The integrated chatbot is constrained to answer questions exclusively from the book's text to ensure accuracy and prevent hallucinations.

### VIII. Content Integrity
All content must be original and unique to this project. Contradictions across chapters are strictly prohibited; the material must serve as a single, cohesive source of truth for the reader.

## Technical Stack
**Frontend/Publication**: Docusaurus (React-based static site generator).
**RAG System**: Python-based backend for embeddings and retrieval.
**Robotics Simulation**: ROS 2, Gazebo, Unity, NVIDIA Isaac Sim.

## Workflow & Quality Assurance
1. **Drafting**: Content is expanded within existing files, following the intuition-to-limitations flow.
2. **Visuals**: Text-based diagrams are created for every lesson.
3. **Verification**: Code examples and technical claims are verified against official docs.
4. **Formatting**: Markdown is validated for Docusaurus compatibility (frontmatter, callouts).
5. **Review**: Content is reviewed for depth, style, and structure compliance.

## Governance
This Constitution supersedes all other project directives.
Amendments to these principles require a documented reasoning and a Semantic Versioning bump.
All contributions (text or code) must be verified against these principles before acceptance.

**Version**: 1.1.0 | **Ratified**: 2025-12-08 | **Last Amended**: 2025-12-13