<!-- SYNC IMPACT REPORT:
Version change: N/A (initial creation) → 1.0.0
Added sections: All principles and sections as specified
Removed sections: None (new file)
Templates requiring updates: ⚠ pending (no existing templates to update)
Follow-up TODOs: None
-->
# Integrated RAG Chatbot Backend for AI/Spec-Driven Book Constitution

## Core Principles

### Groundedness
All responses must be generated exclusively from retrieved book content with no external knowledge sources.
Responses must fail safely when relevant context is not found in the retrieved content, rather than generating hallucinated information.

### Faithfulness
No hallucinations or unsupported claims; responses must be factually accurate to source material.
Every statement in a response must be verifiable against the provided source documents, with explicit prohibition of external knowledge usage.

### Traceability
Every response must be traceable to source chunks with clear citation of document sections.
The system must maintain clear links between query, retrieved documents, and generated response for auditability and transparency.

### Modularity
Clear separation of ingestion, retrieval, ranking, and generation components with well-defined interfaces.
Each component must be independently testable and replaceable to ensure clean architectural boundaries and maintainability.

### Spec-driven Rigor
All behavior derived from explicit specifications with comprehensive validation requirements.
Changes must be driven by formal specifications with corresponding test cases that validate compliance with defined requirements.

### Provider Abstraction
LLM usage via Cohere SDK only (no OpenAI dependencies) with clean interfaces for provider switching.
External service dependencies must be abstracted behind interfaces to allow for easy substitution without affecting core logic.

## Technical Standards and Constraints

### Retrieval Requirements
- Vector storage: Qdrant Cloud (Free Tier)
- Embeddings: Cohere embeddings API
- Chunking strategy must be deterministic and documented
- Metadata must include source path, section, and chunk ID

### Generation Requirements
- LLM provider: Cohere (command / chat models)
- Responses must cite retrieved chunk IDs internally
- System prompt must explicitly prohibit use of external knowledge

### API Requirements
- Framework: FastAPI
- Endpoints:
  - /ingest (book content ingestion)
  - /query (general book QA)
  - /query/selection (answers strictly from user-selected text)
- All inputs and outputs must be schema-validated

### Storage Requirements
- Neon Serverless Postgres for:
  - Document metadata
  - Chat sessions
  - Query logs
- No raw embeddings stored outside Qdrant

### Security & Reliability Requirements
- Rate limiting enabled
- Input sanitization required
- Deterministic temperature settings for reproducibility
- Selected-text queries must ignore the global vector index entirely

## Development Workflow and Quality Assurance

### Code Standards
- Backend-only implementation (frontend handled by Docusaurus)
- All inputs and outputs must be schema-validated
- Comprehensive error handling with clear client-facing messages
- Deterministic behavior where reproducibility is critical

### Testing Requirements
- End-to-end RAG pipeline operational
- Book content successfully ingested and queried
- Selection-based QA working as specified
- Zero hallucinated facts during evaluation

### Deployment and Documentation
- Backend deployable and documented
- Clear, auditable RAG pipeline
- Spec-Kit Plus specifications fully implemented and verifiable

## Governance

This constitution supersedes all other development practices and requirements for the Integrated RAG Chatbot Backend project. All changes to the codebase must comply with these core principles, and any deviation must be explicitly justified and reviewed by project leadership.

Amendments to this constitution require documentation of reasoning, approval from project stakeholders, and a migration plan for existing code that may conflict with new requirements. All pull requests and reviews must verify compliance with these principles before merging.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17