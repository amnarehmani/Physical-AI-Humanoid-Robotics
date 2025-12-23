from datetime import datetime
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from uuid import UUID, uuid4


class RetrievedChunkInfo(BaseModel):
    chunk_id: str
    content: str
    source_document: str
    section_title: str


class QueryLogBase(BaseModel):
    session_id: Optional[str] = Field(None, max_length=128)
    query_text: str
    response_text: str
    retrieved_chunks: Optional[List[RetrievedChunkInfo]] = Field(default_factory=list)
    response_time_ms: Optional[int] = None


class QueryLogCreate(QueryLogBase):
    pass


class QueryLogUpdate(BaseModel):
    response_text: Optional[str] = None
    retrieved_chunks: Optional[List[RetrievedChunkInfo]] = None
    response_time_ms: Optional[int] = None


class QueryLog(QueryLogBase):
    id: UUID = Field(default_factory=uuid4)
    timestamp: datetime = Field(default_factory=datetime.utcnow)
    
    class Config:
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "123e4567-e89b-12d3-a456-426614174003",
                "session_id": "session-123",
                "query_text": "What are the key principles?",
                "response_text": "The key principles include retrieval, generation, and grounding...",
                "retrieved_chunks": [
                    {
                        "chunk_id": "chunk-123",
                        "content": "The first principle is retrieval...",
                        "source_document": "Introduction to RAG Systems",
                        "section_title": "Key Principles"
                    }
                ],
                "timestamp": "2023-01-01T00:00:00.000Z",
                "response_time_ms": 1200
            }
        }