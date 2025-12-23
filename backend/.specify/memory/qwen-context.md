# RAG Chatbot Backend - Technology Context

## Overview
This file contains technology-specific context for the RAG Chatbot Backend project. It's used by AI agents working on this project to understand the technology stack and implementation patterns.

## Technology Stack

### Backend Framework
- **FastAPI**: Modern, fast web framework for building APIs with Python 3.7+ based on standard Python type hints.
- **Uvicorn**: Lightning-fast ASGI server implementation, using uvloop and httptools.

### Vector Database
- **Qdrant Cloud**: Managed vector database service for storing and retrieving embeddings.
- **Qdrant Client**: Python SDK for interacting with Qdrant Cloud instances.

### Database
- **Neon Serverless Postgres**: Serverless PostgreSQL for storing document metadata, chat sessions, and query logs.
- **AsyncPG**: Fast PostgreSQL client library for Python/asyncio.

### LLM & Embeddings
- **Cohere**: NLP platform providing embedding and text generation models.
- **Cohere SDK**: Python SDK for interacting with Cohere's API.

### Data Validation
- **Pydantic**: Data validation and settings management using Python type hints.

### Environment Management
- **python-dotenv**: Reads key-value pairs from a .env file and adds them to the environment variable.

## Key Implementation Patterns

### Ingestion Pipeline
1. Document content is validated and parsed
2. Content is deterministically chunked preserving semantic meaning
3. Embeddings are generated using Cohere
4. Chunks are stored in Qdrant with appropriate metadata
5. Metadata is stored in Neon Postgres for reference

### Query Processing
1. User question is embedded using Cohere
2. Vector search retrieves relevant chunks from Qdrant
3. Context is assembled from retrieved chunks
4. Response is generated using Cohere with book content as context
5. Response is validated to ensure groundedness

### Selection Query Processing
1. Selected text and question bypass vector store completely
2. Response is generated using only provided text
3. Validation ensures response is based only on provided text
4. System fails safely if insufficient information present

### Error Handling
- All API endpoints return structured error responses
- Input validation prevents bad data from entering the system
- External service failures are handled gracefully with fallbacks
- Comprehensive logging for debugging and monitoring

## Architecture Principles
- **Modularity**: Clear separation of ingestion, retrieval, and generation components
- **Groundedness**: All responses must be based exclusively on retrieved book content
- **Traceability**: Every response must be traceable to source chunks
- **Provider Abstraction**: External service dependencies abstracted behind interfaces
- **Security**: Input sanitization and environment variable handling

## API Endpoints
- `POST /ingest`: Ingest book content with deterministic chunking
- `POST /query`: Query book content with vector retrieval
- `POST /query/selection`: Query using only selected text, bypassing vector store