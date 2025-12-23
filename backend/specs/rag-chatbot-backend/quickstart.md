# Quickstart Guide: RAG Chatbot Backend

## Overview
This guide provides a quick setup and initial usage instructions for the RAG Chatbot Backend that enables Q&A over Docusaurus-published book content.

## Prerequisites
- Python 3.9+
- pip package manager
- Docker (optional, for containerized deployment)
- Access to required external services:
  - Qdrant Cloud account
  - Cohere API key
  - Neon Serverless Postgres account

## Setting Up the Environment

### 1. Clone and Navigate to Project
```bash
cd C:\Users\Amna Rehman\OneDrive\Desktop\book\backend
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install fastapi uvicorn cohere qdrant-client asyncpg python-dotenv
```

### 4. Set Up Environment Variables
Create a `.env` file in the project root with the following variables:

```env
NEON_DATABASE_URL=your_neon_database_url
QDRANT_API_KEY=your_qdrant_cloud_api_key
QDRANT_CLUSTER_ID=your_qdrant_cluster_id
QDRANT_URL=your_qdrant_cluster_url
COHERE_API_KEY=your_cohere_api_key
```

## Running the Application

### 1. Start the FastAPI Server
```bash
uvicorn main:app --reload --port 8000
```

### 2. Verify Installation
Open your browser or use curl to check if the server is running:
```bash
curl http://localhost:8000/health
```

## Quick API Tests

### 1. Ingest Sample Content
```bash
curl -X POST "http://localhost:8000/ingest" \
  -H "Content-Type: application/json" \
  -d '{
    "content": "# Sample Book Content\nThis is a sample section from the book...",
    "source_path": "/docs/sample.md",
    "title": "Sample Document"
  }'
```

### 2. Query the Content
```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is this sample about?",
    "session_id": "test-session"
  }'
```

### 3. Query Using Selected Text Only
```bash
curl -X POST "http://localhost:8000/query/selection" \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "This is a sample section from the book that discusses important concepts.",
    "question": "What does this section discuss?"
  }'
```

## Core Components Overview

### 1. Ingestion Pipeline (`ingserter.py`)
Handles the intake of Markdown/MDX book content, splits it into chunks, and stores embeddings in Qdrant while maintaining metadata in Neon.

### 2. Query Processing (`query_service.py`)
Manages the retrieval of relevant content chunks from Qdrant and generates grounded answers using Cohere, ensuring no hallucinations or external knowledge is used.

### 3. API Layer (`main.py`, `routers/`)
FastAPI endpoints that expose the core functionality to external clients with proper request validation and error handling.

### 4. Data Models (`models.py`)
Pydantic models for request/response validation and database schema definitions.

## Configuration Options

### Environment Variables
- `NEON_DATABASE_URL`: Connection string for Neon Postgres database
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_CLUSTER_ID`: Cluster identifier for Qdrant
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `COHERE_API_KEY`: API key for Cohere services
- `MAX_SELECTED_TEXT_LENGTH`: Maximum allowed size for selected text (default: 10KB)

### Runtime Parameters
- `--host`: Host address for the server (default: localhost)
- `--port`: Port for the server (default: 8000)
- `--workers`: Number of worker processes (default: 1 for development)

## Troubleshooting

### Common Issues

1. **Environment Variables Not Loaded**
   - Ensure `.env` file is properly formatted and located in the correct directory
   - Verify all required environment variables are set

2. **Qdrant Connection Issues**
   - Verify QDRANT_URL and QDRANT_API_KEY are correct
   - Check that the Qdrant Cloud cluster is running and accessible

3. **Cohere API Issues**
   - Confirm COHERE_API_KEY is valid and has appropriate permissions
   - Check API rate limits if requests are failing intermittently

4. **Database Connection Issues**
   - Verify NEON_DATABASE_URL is correctly formatted
   - Ensure the Neon Serverless Postgres instance is active

## Next Steps
1. Review the complete API documentation at `/docs` when the server is running
2. Implement the data models and database connection based on the schema
3. Set up the ingestion pipeline with chunking and embedding logic
4. Implement query processing and response generation
5. Add comprehensive error handling and validation
6. Set up proper logging and monitoring