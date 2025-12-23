# Tasks: RAG Chatbot Backend

**Feature**: RAG Chatbot Backend for AI-Spec-Driven Book
**Created**: 2025-12-18
**Status**: Initial
**Spec Reference**: `backend/specs/rag-chatbot-backend.spec.md`
**Plan Reference**: `backend/specs/rag-chatbot-backend/plan.md`

## Dependencies

- **User Story 1 (P1) must complete before User Story 2 (P2)**: Content must be ingested before queries can be processed
- **User Story 2 (P2) and User Story 3 (P3)**: Independent and can be developed in parallel after User Story 1

## Parallel Execution Examples

- **Within User Story 2**: Database models, API endpoints, and service implementations can be developed in parallel by different developers
- **Within User Story 3**: Selection query endpoint and validation logic can be developed independently of global querying

## Implementation Strategy

- **MVP Scope**: Complete User Story 1 (Ingest Book Content) to have a minimal but working system
- **Incremental Delivery**: Each user story builds on the previous to deliver value incrementally

---

## Phase 1: Setup & Project Initialization

- [X] T001 Create project directory structure (src/, tests/, config/, etc.)
- [X] T002 Initialize Python project with pyproject.toml or setup.py
- [X] T003 [P] Install and configure dependencies (FastAPI, Cohere, Qdrant, Neon Postgres)
- [X] T004 [P] Set up environment variables configuration
- [X] T005 Set up basic FastAPI application structure
- [X] T006 Configure logging system
- [X] T007 Create configuration management module
- [X] T008 Set up .gitignore for backend project

## Phase 2: Foundational Components

- [X] T009 Set up database connection abstraction for Neon Postgres
- [X] T010 [P] Create database models for Documents table (Neon Postgres)
- [X] T011 [P] Create database models for Chunks table (Neon Postgres)
- [X] T012 [P] Create database models for Chat Sessions table (Neon Postgres)
- [X] T013 [P] Create database models for Query Logs table (Neon Postgres)
- [X] T014 Set up Qdrant client connection
- [X] T015 Create Qdrant collection for book chunks
- [X] T016 Implement environment variable validation
- [X] T017 Create abstract base classes for service interfaces
- [X] T018 Implement request/response validation models using Pydantic

## Phase 3: [US1] Ingest Book Content (Priority: P1)

### Story Goal
A system administrator needs to ingest book content from a Docusaurus-published book into the RAG system so that users can later query the content for accurate answers.

### Independent Test Criteria
Can be fully tested by calling the POST /ingest endpoint with sample book content and verifying that the content has been successfully stored in both the vector database and metadata store, accessible for querying.

- [X] T019 [P] [US1] Create IngestRequest Pydantic model for API validation
- [X] T020 [P] [US1] Create IngestResponse Pydantic model for API validation
- [X] T021 [P] [US1] Implement Document model CRUD operations for Neon
- [X] T022 [P] [US1] Implement Chunk model CRUD operations for Neon
- [X] T023 [US1] Create deterministic chunking algorithm implementation
- [X] T024 [US1] Implement Cohere embedding generation for text chunks
- [X] T025 [US1] Create VectorStore interface and Qdrant implementation
- [X] T026 [US1] Create MetadataStore interface and Neon implementation
- [X] T027 [US1] Implement IngestionService with complete pipeline logic
- [X] T028 [US1] Create /ingest endpoint in FastAPI router
- [X] T029 [US1] Implement content validation and sanitization for ingestion
- [X] T030 [US1] Add rate limiting for concurrent ingestion operations (max 2)
- [X] T031 [US1] Implement checksum generation for document change detection
- [X] T032 [US1] Add document status tracking (pending, processing, complete, failed)
- [X] T033 [US1] Implement error handling and logging for ingestion process
- [X] T034 [US1] Validate Markdown/MDX format parsing
- [X] T035 [US1] Add content size validation before processing
- [X] T036 [US1] Implement document versioning for updates
- [ ] T037 [US1] Test ingestion with various formatting elements (headings, lists, code blocks)
- [ ] T038 [US1] Verify deterministic chunking preserves semantic meaning
- [ ] T039 [US1] Test ingestion of substantial content (>100 pages equivalent)

## Phase 4: [US2] Query Book Content Globally (Priority: P2)

### Story Goal
A user wants to ask questions about the book content and receive accurate answers based solely on the book's information, ensuring no hallucinations or external knowledge contamination.

### Independent Test Criteria
Can be fully tested by calling the POST /query endpoint with a question and receiving a response that is grounded in the book content, with no external information.

- [X] T040 [P] [US2] Create QueryRequest Pydantic model for API validation
- [X] T041 [P] [US2] Create QueryResponse Pydantic model for API validation
- [X] T042 [P] [US2] Create RetrievedChunk Pydantic model for API validation
- [X] T043 [US2] Implement Cohere embedding generation for query text
- [X] T044 [US2] Implement Qdrant search with appropriate parameters for retrieval
- [ ] T045 [US2] Create Retriever service for chunk retrieval from Qdrant
- [X] T046 [US2] Create Generator service for response generation with Cohere
- [X] T047 [US2] Implement context assembly from retrieved chunks
- [X] T048 [US2] Add internal citation of chunk IDs to responses
- [X] T049 [US2] Create QueryService orchestrating query processing
- [X] T050 [US2] Implement /query endpoint in FastAPI router
- [X] T051 [US2] Add response validation to ensure groundedness
- [X] T052 [US2] Implement confidence scoring for responses
- [X] T053 [US2] Add query logging to Query Logs table
- [X] T054 [US2] Implement hallucination detection logic
- [X] T055 [US2] Set up deterministic Cohere generation settings for consistency
- [ ] T056 [US2] Test query with content that exists in book (accuracy verification)
- [ ] T057 [US2] Test query with content not in book (acknowledge insufficient info)
- [ ] T058 [US2] Test synthesis of information from multiple relevant chunks
- [ ] T059 [US2] Verify response time under 5 seconds requirement

## Phase 5: [US3] Query Using Selected Text Only (Priority: P3)

### Story Goal
A user has selected specific text from the book interface and wants to ask questions about only that selected text, bypassing the broader vector store entirely.

### Independent Test Criteria
Can be fully tested by calling the POST /query/selection endpoint with selected text and a question, receiving a response that only considers the provided text segment.

- [X] T060 [P] [US3] Create SelectionQueryRequest Pydantic model for API validation
- [X] T061 [P] [US3] Create SelectionQueryResponse Pydantic model for API validation
- [X] T062 [US3] Implement /query/selection endpoint in FastAPI router
- [X] T063 [US3] Create validation for selected text size (max 10KB)
- [X] T064 [US3] Implement response generation using only provided text
- [X] T065 [US3] Add logic to bypass vector store entirely for selection queries
- [X] T066 [US3] Implement safe failure when selected text has insufficient information
- [X] T067 [US3] Add confidence indicator to selection query responses
- [X] T068 [US3] Validate that response is based only on provided text
- [X] T069 [US3] Implement validation for query against selected text relevance
- [ ] T070 [US3] Test with selected text that contains information to answer question
- [ ] T071 [US3] Test with selected text that doesn't contain information to answer
- [ ] T072 [US3] Test with ambiguous references in selected text

## Phase 6: Polish & Cross-Cutting Concerns

- [ ] T073 Implement comprehensive error handling with structured responses
- [ ] T074 Add request/response logging for debugging and monitoring
- [ ] T075 Implement health check endpoint
- [ ] T076 Add API documentation with Swagger/OpenAPI
- [ ] T077 Create system-wide validation of environment variables at startup
- [ ] T078 Implement performance monitoring and metrics
- [ ] T079 Add input sanitization across all endpoints
- [ ] T080 Create configuration for production deployment
- [ ] T081 Implement graceful shutdown procedures
- [ ] T082 Create backup and recovery procedures for metadata
- [ ] T083 Add comprehensive unit tests for all services
- [ ] T084 Add integration tests for API endpoints
- [ ] T085 Add end-to-end tests for complete user journeys
- [ ] T086 Validate all responses meet groundedness and hallucination requirements
- [ ] T087 Verify 99% reliability for content ingestion over extended operation
- [ ] T088 Confirm 95% accuracy for queries with zero hallucinations
- [ ] T089 Ensure 100% compliance for selected-text queries using only provided text
- [ ] T090 Verify reproducible behavior with consistent responses to identical queries