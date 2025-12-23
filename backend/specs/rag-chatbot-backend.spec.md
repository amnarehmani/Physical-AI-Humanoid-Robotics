# Feature Specification: RAG Chatbot Backend

**Feature Branch**: `rag-chatbot-backend`
**Created**: 2025-12-18
**Status**: Draft
**Input**: User description: "RAG Chatbot Backend for AI-Spec-Driven Book Workspace: - This specification belongs to the BACKEND workspace - All spec artifacts MUST be created under: backend/specs/ - Do NOT create or modify any files or folders outside backend/ - Do NOT treat book modules or repo root as spec workspace Target audience: - Backend developers implementing the RAG system - Reviewers validating backend correctness and grounding behavior Focus: - Retrieval-Augmented Generation (RAG) backend - Question answering over a Docusaurus-published book - Strictly grounded responses using book content only - Support for both global book queries and selected-text-only queries Success criteria: - Book content can be ingested, embedded, and stored successfully - Global queries return accurate, grounded answers from retrieved chunks - Selected-text queries answer strictly from provided text only - No hallucinations or use of external knowledge - Backend is stable, reproducible, and spec-complete Functional scope: - Backend-only implementation - API-driven architecture using FastAPI - Vector retrieval using Qdrant Cloud - Metadata and session storage using Neon Serverless Postgres - LLM inference and embeddings using Cohere - Developed using Spec-Kit Plus with Gemini CLI Constraints: - No OpenAI SDKs or APIs - No web search or external browsing - No frontend or UI components - Deterministic generation settings (low temperature) - All secrets loaded via environment variables only Required environment variables: - NEON_DATABASE_URL - QDRANT_API_KEY - QDRANT_CLUSTER_ID - QDRANT_URL - COHERE_API_KEY API requirements: - POST /ingest - Ingest Markdown/MDX book content - Deterministic chunking - Generate embeddings - Store vectors in Qdrant - Store metadata in Neon - POST /query - Accept user question - Retrieve relevant chunks from Qdrant - Generate grounded response using Cohere - POST /query/selection - Accept user-selected text and question - Bypass vector store entirely - Restrict context strictly to selected text - Fail safely if insufficient information is present Non-functional requirements: - Input validation and sanitization - Structured error handling - Modular and testable components - Clear separation of ingestion, retrieval, and generation layers - Qdrant Free Tier compatible - Neon Serverless compatible Not building: - Frontend UI or chat widgets - Authentication or user management - Analytics or logging dashboards - Multi-book or multi-tenant support - Model fine-tuning pipelines"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ingest Book Content (Priority: P1)

A system administrator needs to ingest book content from a Docusaurus-published book into the RAG system so that users can later query the content for accurate answers. This involves uploading Markdown/MDX files, processing them into chunks, and storing embeddings in the vector database.

**Why this priority**: Without ingested content, the entire system has no knowledge base to draw from, making it impossible to answer user queries. This foundational capability enables all downstream functionality.

**Independent Test**: Can be fully tested by calling the POST /ingest endpoint with sample book content and verifying that the content has been successfully stored in both the vector database and metadata store, accessible for querying.

**Acceptance Scenarios**:

1. **Given** an authenticated administrator uploads book content, **When** the system processes the content with the /ingest endpoint, **Then** the content is chunked deterministically and embeddings are stored in Qdrant and metadata in Neon
2. **Given** book content contains various formatting elements (headings, lists, code blocks), **When** ingesting, **Then** the system preserves semantic meaning during chunking and stores properly formatted content
3. **Given** the system is under normal load conditions, **When** ingesting substantial book content (100+ pages), **Then** the process completes within a reasonable timeframe and doesn't exceed resource limits

---

### User Story 2 - Query Book Content Globally (Priority: P2)

A user wants to ask questions about the book content and receive accurate answers based solely on the book's information, ensuring no hallucinations or external knowledge contamination.

**Why this priority**: This is the core value proposition of the RAG system. Users need to be able to get reliable answers to their questions directly from the book content.

**Independent Test**: Can be fully tested by calling the POST /query endpoint with a question and receiving a response that is grounded in the book content, with no external information.

**Acceptance Scenarios**:

1. **Given** user submits a question about book content, **When** the system retrieves relevant chunks from Qdrant and processes with Cohere, **Then** the response contains accurate information from the book without hallucinations
2. **Given** user asks a question not covered by book content, **When** the system processes the query, **Then** the response indicates insufficient information is available rather than making things up
3. **Given** multiple relevant chunks exist in the vector store, **When** the system generates a response, **Then** it synthesizes information coherently from multiple sources within the book

---

### User Story 3 - Query Using Selected Text Only (Priority: P3)

A user has selected specific text from the book interface and wants to ask questions about only that selected text, bypassing the broader vector store entirely.

**Why this priority**: This provides users with more focused answers based on specific passages they're reading, rather than getting responses from the entire book that might be less relevant.

**Independent Test**: Can be fully tested by calling the POST /query/selection endpoint with selected text and a question, receiving a response that only considers the provided text segment.

**Acceptance Scenarios**:

1. **Given** user provides selected text and a related question, **When** the system processes the request with /query/selection, **Then** the response is generated using only the provided text without accessing the vector store
2. **Given** user selects text that doesn't contain information to answer the question, **When** processing with /query/selection, **Then** the system acknowledges insufficient information in the selected text
3. **Given** selected text contains ambiguous references, **When** generating a response, **Then** the system clarifies based only on the provided text context

---

### Edge Cases

- What happens when a query is submitted but the vector store is temporarily unavailable?
- How does the system handle extremely long selected text segments in the selection query endpoint?
- What occurs if ingested content contains malformed Markdown or code snippets?
- How does the system respond to queries that appear to be testing its grounding (e.g., "What's your opinion on X?" when opinions aren't in the book)?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST accept Markdown/MDX book content via POST /ingest endpoint
- **FR-002**: System MUST perform deterministic chunking of ingested content to enable consistent retrieval
- **FR-003**: System MUST generate embeddings for content chunks and store them in vector database
- **FR-004**: System MUST store metadata about ingested content in Neon Serverless Postgres
- **FR-005**: System MUST accept user questions via POST /query endpoint and respond with grounded answers
- **FR-006**: System MUST retrieve relevant content chunks from vector store for query processing
- **FR-007**: System MUST accept user-selected text and questions via POST /query/selection endpoint
- **FR-008**: System MUST process selection queries without accessing the vector store, using only provided text
- **FR-009**: System MUST fail safely when selected text does not contain sufficient information to answer a question
- **FR-010**: System MUST prevent hallucinations by restricting responses to the provided book content
- **FR-011**: System MUST validate and sanitize all incoming content and queries
- **FR-012**: System MUST handle errors gracefully with structured error responses

### Key Entities

- **Book Content**: Represents the original documentation content in Markdown/MDX format, including metadata about source files
- **Text Chunk**: A segment of processed book content that has been split according to deterministic rules, with associated embedding vectors
- **Query Response**: A text response generated for a user question, grounded in specific book content chunks, with confidence indicators

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Book content can be ingested, embedded, and stored successfully with 99% reliability over 30 days of continuous operation
- **SC-002**: Global queries return accurate, grounded answers from retrieved chunks within 5 seconds average response time
- **SC-003**: At least 95% of queries result in responses that are factually accurate according to the book content, with zero hallucinations
- **SC-004**: Selected-text queries answer strictly from provided text only, with no reference to external knowledge, achieving 100% compliance
- **SC-005**: System demonstrates reproducible behavior with consistent responses to identical queries over time