# Data Model: RAG Chatbot Backend

## Database Schema (Neon Serverless Postgres)

### 1. Documents Table
**Purpose**: Store metadata about ingested documents

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the document |
| title | VARCHAR(255) | NOT NULL | Document title |
| source_path | VARCHAR(500) | NOT NULL | Path from which the document was sourced |
| checksum | VARCHAR(64) | NOT NULL | SHA-256 hash of document content for change detection |
| version | INTEGER | NOT NULL, DEFAULT 1 | Version number for tracking updates |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Creation timestamp |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last update timestamp |
| status | VARCHAR(20) | NOT NULL, CHECK(status IN ('pending', 'processing', 'completed', 'failed')) | Current ingestion status |
| chunk_count | INTEGER | NOT NULL, DEFAULT 0 | Number of chunks created from this document |

### 2. Chunks Table
**Purpose**: Store metadata about text chunks created during ingestion

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the chunk |
| document_id | UUID | FOREIGN KEY, NOT NULL | Reference to parent document |
| chunk_order | INTEGER | NOT NULL | Order of the chunk in the original document |
| chunk_id | VARCHAR(128) | NOT NULL | Qdrant Point ID for this chunk |
| content_preview | TEXT | NOT NULL | First 500 characters of chunk content |
| section_title | VARCHAR(255) | | Title of the section this chunk belongs to |
| metadata | JSONB | | Additional metadata related to the chunk |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Creation timestamp |

### 3. Chat Sessions Table
**Purpose**: Track chat sessions for potential future analytics or continuity

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the session |
| session_id | VARCHAR(128) | NOT NULL, UNIQUE | Client-provided session identifier |
| created_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Session creation timestamp |
| updated_at | TIMESTAMP | NOT NULL, DEFAULT NOW() | Last interaction timestamp |
| last_query | TEXT | | The most recent query in this session |

### 4. Query Logs Table
**Purpose**: Store query logs for debugging, analytics, and compliance

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, NOT NULL | Unique identifier for the log entry |
| session_id | VARCHAR(128) | | Reference to the chat session |
| query_text | TEXT | NOT NULL | The original user query |
| response_text | TEXT | NOT NULL | The system's response |
| retrieved_chunks | JSONB | | List of chunk IDs and metadata used to generate the response |
| timestamp | TIMESTAMP | NOT NULL, DEFAULT NOW() | When the query was processed |
| response_time_ms | INTEGER | | Time taken to generate the response in milliseconds |

## Vector Store Schema (Qdrant Cloud)

### Collection: `book_chunks`

#### Vector Configuration:
- **Size**: 1024 (Cohere multilingual embedding dimension)
- **Distance**: Cosine

#### Payload Structure:
```json
{
  "document_id": "uuid",
  "chunk_id": "string",
  "section_title": "string",
  "source_path": "string", 
  "content_preview": "string",
  "chunk_order": "integer",
  "metadata": {
    "paragraph_count": "integer",
    "word_count": "integer",
    "language": "string",
    "tags": ["string"]
  }
}
```

## API Request/Response Schemas

### 1. Ingestion Endpoint: POST /ingest

#### Request Schema
```json
{
  "content": "string",
  "source_path": "string",
  "title": "string"
}
```

#### Response Schema (Success)
```json
{
  "status": "success",
  "message": "string",
  "document_id": "uuid",
  "chunks_processed": "integer"
}
```

#### Response Schema (Error)
```json
{
  "status": "error",
  "message": "string",
  "error_code": "string"
}
```

### 2. Global Query Endpoint: POST /query

#### Request Schema
```json
{
  "question": "string",
  "session_id": "string"
}
```

#### Response Schema (Success)
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

### 3. Selection Query Endpoint: POST /query/selection

#### Request Schema
```json
{
  "selected_text": "string",
  "question": "string"
}
```

#### Response Schema (Success)
```json
{
  "question": "string",
  "answer": "string",
  "confidence": "enum: high | medium | low",
  "sufficient_information": "boolean"
}
```

## Relationships

1. **Documents → Chunks**: One-to-many (One document generates many chunks)
2. **Chat Sessions → Query Logs**: One-to-many (One session may have multiple queries)
3. **Chunks ↔ Qdrant**: The chunk_id in the database corresponds to the Point ID in Qdrant