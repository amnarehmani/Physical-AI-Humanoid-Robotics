from datetime import datetime
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field
from uuid import UUID, uuid4


class ChunkBase(BaseModel):
    document_id: UUID
    chunk_order: int
    chunk_id: str = Field(..., max_length=128)  # Qdrant Point ID
    content_preview: str
    section_title: Optional[str] = Field(None, max_length=255)
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)


class ChunkCreate(ChunkBase):
    pass


class ChunkUpdate(BaseModel):
    content_preview: Optional[str] = None
    section_title: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = None


class Chunk(ChunkBase):
    id: UUID = Field(default_factory=uuid4)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    
    class Config:
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "123e4567-e89b-12d3-a456-426614174001",
                "document_id": "123e4567-e89b-12d3-a456-426614174000",
                "chunk_order": 1,
                "chunk_id": "chunk-123",
                "content_preview": "This is the beginning of the RAG system...",
                "section_title": "Introduction",
                "metadata": {"paragraph_count": 5, "word_count": 120},
                "created_at": "2023-01-01T00:00:00.000Z"
            }
        }