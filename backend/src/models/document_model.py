from datetime import datetime
from typing import Optional
from enum import Enum
from pydantic import BaseModel, Field
from uuid import UUID, uuid4


class DocumentStatus(str, Enum):
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"


class DocumentBase(BaseModel):
    title: str = Field(..., max_length=255)
    source_path: str = Field(..., max_length=500)
    checksum: str = Field(..., max_length=64)
    version: int = Field(default=1, ge=1)


class DocumentCreate(DocumentBase):
    """Schema for creating a new document"""
    pass


class DocumentUpdate(BaseModel):
    """Schema for updating an existing document"""
    title: Optional[str] = Field(None, max_length=255)
    status: Optional[DocumentStatus] = None
    chunk_count: Optional[int] = None


class Document(DocumentBase):
    """Full document model including metadata"""
    id: UUID = Field(default_factory=uuid4)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    status: DocumentStatus = Field(default=DocumentStatus.PENDING)
    chunk_count: int = Field(default=0)
    
    class Config:
        from_attributes = True
        json_schema_extra = {
            "example": {
                "id": "123e4567-e89b-12d3-a456-426614174000",
                "title": "Introduction to RAG Systems",
                "source_path": "/docs/intro.md",
                "checksum": "sha256_hash_here",
                "version": 1,
                "created_at": "2023-01-01T00:00:00.000Z",
                "updated_at": "2023-01-01T00:00:00.000Z",
                "status": "completed",
                "chunk_count": 25
            }
        }
