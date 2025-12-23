# RAG Chatbot Backend

This is a Retrieval-Augmented Generation (RAG) backend that enables Q&A over Docusaurus-published book content. The system allows users to ask questions about book content and receive answers grounded exclusively in the book's information, with no hallucinations or external knowledge usage.

## Features

- **Document Ingestion**: Accepts Markdown/MDX book content, processes it into chunks, and stores embeddings
- **Global Querying**: Allows users to ask questions about the entire book content
- **Selection-Based Querying**: Allows users to ask questions based only on selected text
- **Grounded Responses**: All answers are strictly based on retrieved book content
- **No Hallucinations**: System prevents generating information not present in the source

## Architecture

The system is built with these main components:

- **API Layer**: FastAPI endpoints for ingestion and querying
- **Ingestion Pipeline**: Processes documents into chunks and generates embeddings
- **Vector Storage**: Qdrant Cloud for document chunk embeddings
- **Metadata Storage**: Neon Serverless Postgres for document metadata
- **Query Processing**: Retrieves relevant chunks and generates responses using Cohere

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables in a `.env` file:
   ```env
   NEON_DATABASE_URL=your_neon_database_url
   QDRANT_API_KEY=your_qdrant_cloud_api_key
   QDRANT_CLUSTER_ID=your_qdrant_cluster_id
   QDRANT_URL=your_qdrant_cluster_url
   COHERE_API_KEY=your_cohere_api_key
   ```

3. Run the application:
   ```bash
   uvicorn src.main:app --reload
   ```

## API Endpoints

- `POST /api/v1/ingest` - Ingest book content
- `POST /api/v1/query` - Query book content globally
- `POST /api/v1/query/selection` - Query using selected text only
- `GET /health` - Health check

## Environment Variables

- `NEON_DATABASE_URL`: Connection string for Neon Postgres database
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_CLUSTER_ID`: Cluster identifier for Qdrant
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `COHERE_API_KEY`: API key for Cohere services
- `MAX_SELECTED_TEXT_LENGTH`: Maximum allowed size for selected text (default: 10240 bytes)
- `MAX_CONCURRENT_INGEST`: Maximum concurrent ingestion operations (default: 2)
- `MAX_BOOK_PAGES`: Maximum book size in pages (default: 1000)
- `RESPONSE_TIMEOUT`: Query response timeout in ms (default: 5000)
- `MAX_RETRIEVAL_RESULTS`: Maximum number of results to retrieve (default: 10)

## Project Structure

```
backend/
├── src/
│   ├── config.py          # Configuration and settings
│   ├── main.py            # Main FastAPI application
│   ├── models/            # Pydantic models
│   ├── services/          # Business logic services
│   │   ├── chunker_service.py
│   │   ├── embedder_service.py
│   │   ├── ingestion_service.py
│   │   ├── qdrant_service.py
│   │   ├── query_service.py
│   │   └── neon_metadata_service.py
│   ├── routers/           # API routes
│   │   ├── ingest_router.py
│   │   └── query_router.py
│   └── utils/             # Utility functions
│       └── logging_config.py
├── specs/                 # Specifications and plans
├── tests/                 # Test files
├── pyproject.toml         # Project dependencies
└── requirements.txt       # Python dependencies
```

## Testing

Run tests with pytest:

```bash
pytest tests/
```

## Security

- Input validation and sanitization on all endpoints
- Environment variables for sensitive data
- Rate limiting for ingestion operations
- Size limits on selected text queries