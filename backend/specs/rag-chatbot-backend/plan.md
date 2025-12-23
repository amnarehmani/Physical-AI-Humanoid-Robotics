# Implementation Plan: RAG Chatbot Backend

**Feature**: RAG Chatbot Backend for AI-Spec-Driven Book
**Created**: 2025-12-18
**Status**: Draft
**Spec Reference**: `backend/specs/rag-chatbot-backend.spec.md`

## Technical Context

This plan details the implementation of a Retrieval-Augmented Generation (RAG) backend that powers an embedded chatbot inside a Docusaurus-published book. The system will allow users to ask questions about book content and receive answers grounded exclusively in the book's information, with no hallucinations or external knowledge usage.

### Core Architecture Components
- **Ingestion Pipeline**: For processing and storing book content
- **Vector Storage**: Qdrant Cloud for document chunk embeddings
- **Metadata Storage**: Neon Serverless Postgres for document metadata
- **Query Processing**: For handling global and selection-based queries
- **Generation Engine**: Using Cohere to generate grounded responses
- **API Layer**: FastAPI endpoints for ingestion and querying

### Dependencies
- Cohere SDK: For embeddings and text generation
- Qdrant Client: For vector storage and retrieval
- Neon Postgres: For metadata persistence
- FastAPI: For API endpoint implementation
- Pydantic: For request/response validation

### Known Specifications (from research.md)
- Selected text size limit: 10KB maximum for /query/selection endpoint
- Concurrent ingest operations: Maximum 2 concurrent operations with rate limiting
- Maximum book content size: Support for books up to 1000 pages with adaptive chunking
- Document update approach: Full re-ingestion with versioning (no incremental updates)

## Constitution Check

This implementation plan aligns with the project constitution in the following ways:

### Groundedness
- All responses will be generated exclusively from retrieved book content
- The system will fail safely when relevant context is not found in the retrieved content
- No external knowledge sources will be used

### Faithfulness
- No hallucinations or unsupported claims will be made
- Responses will be factually accurate to source material
- Every statement in a response will be verifiable against source documents

### Traceability
- Every response will be traceable to source chunks with citation of document sections
- Clear links between query, retrieved documents, and generated response will be maintained

### Modularity
- Clear separation of ingestion, retrieval, ranking, and generation components
- Well-defined interfaces between components
- Each component will be independently testable and replaceable

### Spec-driven Rigor
- All behavior will be derived from explicit specifications
- Comprehensive validation requirements will be implemented

### Provider Abstraction
- LLM usage will be via Cohere SDK only (no OpenAI dependencies)
- External service dependencies will be abstracted behind interfaces

## Gates

### Gate 1: Requirements Validation
- [ ] All functional requirements from the spec are addressed in the plan
- [ ] All success criteria from the spec are addressed in the plan
- [ ] All constraints from the spec are respected in the plan

### Gate 2: Architecture Alignment
- [ ] Architecture design aligns with constitution principles
- [ ] Technical choices align with constitution constraints
- [ ] Security and reliability requirements are addressed

### Gate 3: Implementation Feasibility
- [ ] All implementation steps are technically feasible
- [ ] Dependencies are properly scoped and validated
- [ ] Error handling and fallback strategies are defined

---

## Phase 0: Research & Unknown Resolution

### Research Task 1: Selected Text Size Limits
**Issue**: Maximum size limit for selected text in the /query/selection endpoint
**Decision**: 10KB limit for selected text
**Rationale**: This allows for substantial text selections while preventing resource exhaustion. A 10KB limit is reasonable for the amount of text a user might select in a document and should fit within typical LLM context windows
**Alternatives**: 1KB, 5KB, 10KB, 50KB

### Research Task 2: Concurrent Ingest Operations
**Issue**: Handling concurrent ingest operations
**Decision**: Implement rate limiting with a maximum of 2 concurrent ingestion operations
**Rationale**: This protects system resources while still allowing for some parallelism. Ingestion is resource-intensive, so limiting concurrency prevents system overload while maintaining reasonable throughput for multiple documents
**Alternatives**: Sequential processing, rate limiting, concurrent processing with locks

### Research Task 3: Maximum Book Content Size
**Issue**: Expected maximum size of book content to ingest
**Decision**: Support for books up to 1000 pages with adaptive chunking
**Rationale**: This covers the majority of technical and documentation books while requiring adaptive chunking strategies for larger volumes. The system will use different chunking strategies based on document size
**Alternatives**: Small books (<100 pages), Medium books (100-500 pages), Large books (500+ pages)

### Research Task 4: Incremental Document Updates
**Issue**: Support for incremental updates to existing documents
**Decision**: Implement full re-ingestion approach with versioning
**Rationale**: Full re-ingestion is simpler to implement and ensures consistency. It avoids complex change detection algorithms and guarantees that the vector store and metadata are always in sync with the source document
**Alternatives**: Full re-ingestion, incremental updates, no updates allowed

**Research Output**: `research.md`

## Phase 1: Design & Contracts

### Data Model Design

#### Document Metadata Table (Neon Postgres)
- `id` (UUID, Primary Key)
- `title` (VARCHAR, 255)
- `source_path` (VARCHAR, 500)
- `checksum` (VARCHAR, 64)
- `version` (INTEGER)
- `created_at` (TIMESTAMP)
- `updated_at` (TIMESTAMP)
- `status` (ENUM: 'pending', 'processing', 'completed', 'failed')
- `chunk_count` (INTEGER)

#### Chunk Metadata Table (Neon Postgres)
- `id` (UUID, Primary Key)
- `document_id` (UUID, Foreign Key to documents)
- `chunk_order` (INTEGER)
- `chunk_id` (VARCHAR, 128) - Qdrant Point ID
- `content_preview` (TEXT)
- `section_title` (VARCHAR, 255)
- `metadata` (JSONB)
- `created_at` (TIMESTAMP)

#### Chat Sessions Table (Neon Postgres)
- `id` (UUID, Primary Key)
- `session_id` (VARCHAR, 128)
- `created_at` (TIMESTAMP)
- `updated_at` (TIMESTAMP)
- `last_query` (TEXT)

#### Query Logs Table (Neon Postgres)
- `id` (UUID, Primary Key)
- `session_id` (VARCHAR, 128)
- `query_text` (TEXT)
- `response_text` (TEXT)
- `retrieved_chunks` (JSONB)
- `timestamp` (TIMESTAMP)
- `response_time_ms` (INTEGER)

### API Contracts

#### Ingestion Endpoint: POST /ingest
**Request Body**:
```json
{
  "content": "string",
  "source_path": "string",
  "title": "string"
}
```

**Response**:
```json
{
  "status": "success | error",
  "message": "string",
  "document_id": "uuid",
  "chunks_processed": "integer"
}
```

#### Global Query Endpoint: POST /query
**Request Body**:
```json
{
  "question": "string",
  "session_id": "string (optional)"
}
```

**Response**:
```json
{
  "question": "string",
  "answer": "string",
  "retrieved_chunks": [
    {
      "chunk_id": "string",
      "content": "string",
      "source_document": "string",
      "section_title": "string"
    }
  ],
  "confidence": "enum: high | medium | low"
}
```

#### Selection Query Endpoint: POST /query/selection
**Request Body**:
```json
{
  "selected_text": "string",
  "question": "string"
}
```

**Response**:
```json
{
  "question": "string",
  "answer": "string",
  "confidence": "enum: high | medium | low",
  "sufficient_information": "boolean"
}
```

### Qdrant Vector Schema
- **Collection**: `book_chunks`
- **Vector Size**: 1024 (Cohere multi- lingual embedding dimension)
- **Payload Structure**:
```json
{
  "document_id": "uuid",
  "chunk_id": "string",
  "section_title": "string",
  "source_path": "string",
  "content_preview": "string",
  "chunk_order": "integer"
}
```

## Phase 2: Component Architecture

### 1. Ingestion Pipeline
**Components:**
- `IngestionService`: Coordinates the ingestion process
- `Chunker`: Splits documents into deterministic chunks
- `Embedder`: Generates embeddings using Cohere
- `VectorStore`: Manages Qdrant operations
- `MetadataStore`: Manages Neon operations

**Data Flow:**
1. Document content is received via the /ingest endpoint
2. Content is validated for format and size
3. Content is chunked using deterministic rules
4. Embeddings are generated for each chunk
5. Chunks are stored in Qdrant with metadata
6. Chunk metadata is stored in Neon
7. Status is updated in the documents table

### 2. Query Processing
**Components:**
- `QueryService`: Coordinates the query process
- `Retriever`: Retrieves relevant chunks from Qdrant
- `Generator`: Generates responses using Cohere
- `Validator`: Ensures responses are grounded in content

**Data Flow (Global Query):**
1. User question is received via the /query endpoint
2. Question is embedded using Cohere
3. Relevant chunks are retrieved from Qdrant
4. Context is assembled from retrieved chunks
5. Response is generated using Cohere with book content as context
6. Response is validated to ensure groundedness
7. Response is returned with retrieved chunk references

**Data Flow (Selection Query):**
1. Selected text and question received via /query/selection
2. System bypasses vector store completely
3. Response is generated using only the provided text
4. Response is validated to ensure it's based only on provided text
5. Response is returned with confidence indicator

### 3. API Layer
**Components:**
- `app`: FastAPI application instance
- `ingest_router`: Handles /ingest endpoints
- `query_router`: Handles /query endpoints
- `validation_middleware`: Validates requests and sanitizes inputs
- `error_handlers`: Handles error responses

## Phase 3: Implementation Plan

### Sprint 1: Infrastructure and Core APIs
**Objective**: Set up basic project structure and core API endpoints

**Tasks:**
1. Set up FastAPI project structure
   - Create project directory structure
   - Set up requirements.txt with dependencies
   - Configure environment variables handling
   - Set up basic configuration management

2. Implement API endpoints
   - Create /ingest endpoint
   - Create /query endpoint
   - Create /query/selection endpoint
   - Set up request/response validation using Pydantic

3. Implement basic error handling
   - Create error response models
   - Set up exception handlers
   - Implement logging setup

### Sprint 2: Ingestion Pipeline
**Objective**: Implement the complete document ingestion pipeline

**Tasks:**
1. Implement document model and database integration
   - Set up Neon connection
   - Create document and chunk metadata schemas
   - Implement CRUD operations for documents and chunks

2. Implement chunking logic
   - Create deterministic chunking algorithm
   - Handle different content formats (Markdown, MDX)
   - Preserve semantic meaning during chunking
   - Add metadata to each chunk

3. Implement embedding and vector storage
   - Set up Cohere client for embeddings
   - Implement embedding generation for chunks
   - Set up Qdrant connection
   - Implement vector storage with appropriate metadata

### Sprint 3: Query and Generation
**Objective**: Implement query processing and response generation

**Tasks:**
1. Implement retrieval logic
   - Set up query embedding generation
   - Implement Qdrant search with appropriate parameters
   - Handle retrieval results and chunk selection

2. Implement response generation
   - Set up Cohere client for text generation
   - Create system prompts that enforce grounded responses
   - Implement context assembly from retrieved chunks
   - Add internal citation of chunk IDs

3. Implement selection-based query
   - Create endpoint that bypasses vector store
   - Validate selected text and question
   - Generate response based only on provided text
   - Implement safe failure when information is insufficient

### Sprint 4: Validation and Testing
**Objective**: Ensure all system requirements are met and validated

**Tasks:**
1. Implement validation mechanisms
   - Groundedness verification
   - Hallucination detection
   - Spec compliance checks

2. Create comprehensive tests
   - Unit tests for all components
   - Integration tests for API endpoints
   - End-to-end tests for ingestion and querying
   - Performance tests for response times

3. Implement monitoring and observability
   - Add metrics for key operations
   - Set up response time monitoring
   - Add detailed logging for debugging

## Phase 4: Quality Assurance

### Testing Plan
1. **Unit Tests**
   - Test individual components in isolation
   - Validate chunking algorithms
   - Verify embedding generation
   - Test API endpoint validation

2. **Integration Tests**
   - Test complete ingestion pipeline
   - Test query and retrieval pipeline
   - Test selection-based query flow
   - Validate database and vector store operations

3. **End-to-End Tests**
   - Test complete user journeys from ingestion to querying
   - Validate response groundedness
   - Test error conditions and fallbacks
   - Verify performance requirements

### Validation Plan
1. **Groundedness Verification**
   - Check that responses contain only information from book content
   - Verify that no external knowledge is used
   - Test edge cases where content is insufficient

2. **Hallucination Detection**
   - Implement checks to identify made-up information
   - Verify citation of sources within responses
   - Test for factual accuracy of responses

3. **Performance Validation**
   - Measure response times for queries
   - Validate that responses return within 5 seconds (per spec)
   - Test system capacity with multiple concurrent requests

## Phase 5: Deployment & Operations

### Configuration Management
1. Environment variable setup for all required services
   - NEON_DATABASE_URL
   - QDRANT_API_KEY
   - QDRANT_CLUSTER_ID
   - QDRANT_URL
   - COHERE_API_KEY

2. Deployment configuration
   - Containerization (Docker) if needed
   - Health check endpoints
   - Resource allocation settings

### Operational Readiness
1. Logging and monitoring setup
   - Query response time tracking
   - Error rate monitoring
   - Usage metrics

2. Backup and recovery
   - Metadata backup from Neon
   - Vector store backup strategies
   - Document re-ingestion process

## Definition of Done

### Functional Requirements Met
- [ ] FR-001: System accepts Markdown/MDX book content via POST /ingest endpoint
- [ ] FR-002: System performs deterministic chunking of ingested content
- [ ] FR-003: System generates embeddings for content chunks and stores them in vector database
- [ ] FR-004: System stores metadata about ingested content in Neon Serverless Postgres
- [ ] FR-005: System accepts user questions via POST /query endpoint and responds with grounded answers
- [ ] FR-006: System retrieves relevant content chunks from vector store for query processing
- [ ] FR-007: System accepts user-selected text and questions via POST /query/selection endpoint
- [ ] FR-008: System processes selection queries without accessing the vector store
- [ ] FR-009: System fails safely when selected text does not contain sufficient information
- [ ] FR-010: System prevents hallucinations by restricting responses to book content
- [ ] FR-011: System validates and sanitizes all incoming content and queries
- [ ] FR-012: System handles errors gracefully with structured error responses

### Success Criteria Met
- [ ] SC-001: Book content can be ingested, embedded, and stored successfully with 99% reliability
- [ ] SC-002: Global queries return accurate answers within 5 seconds average response time
- [ ] SC-003: At least 95% of queries result in factually accurate responses with zero hallucinations
- [ ] SC-004: Selected-text queries answer strictly from provided text only with 100% compliance
- [ ] SC-005: System demonstrates reproducible behavior with consistent responses

### Technical Standards Met
- [ ] All responses are grounded exclusively in book content
- [ ] No external knowledge sources are used
- [ ] Responses can be traced to specific source chunks
- [ ] Clean separation of ingestion, retrieval, and generation layers
- [ ] All behavior is spec-compliant
- [ ] All dependencies are properly abstracted
- [ ] Input validation and sanitization implemented
- [ ] Proper error handling with clear messages
- [ ] Deterministic behavior for reproducibility