from typing import List, Optional
from pydantic import BaseModel, Field, validator
from uuid import UUID

from ..models.document_model import Document
from ..models.chunk_model import Chunk


class IngestRequest(BaseModel):
    content: str = Field(..., min_length=1, max_length=1000000)  # Up to ~1MB
    source_path: str = Field(..., max_length=500, pattern=r'^\/.*\.mdx?$')
    title: str = Field(..., min_length=1, max_length=255)


class IngestResponse(BaseModel):
    status: str = Field(..., pattern=r'^(success|error)$')
    message: str
    document_id: Optional[UUID] = None
    chunks_processed: Optional[int] = None


class RetrievedChunkResponse(BaseModel):
    chunk_id: str
    content: str
    source_document: str
    section_title: str


class QueryRequest(BaseModel):
    question: str = Field(..., min_length=1, max_length=1000)
    session_id: Optional[str] = Field(None, max_length=128)


class QueryResponse(BaseModel):
    question: str
    answer: str
    retrieved_chunks: List[RetrievedChunkResponse]
    confidence: str = Field(..., pattern=r'^(high|medium|low)$')


class SelectionQueryRequest(BaseModel):
    selected_text: str = Field(..., min_length=1, max_length=10240)  # 10KB limit
    question: str = Field(..., min_length=1, max_length=1000)

    @validator('selected_text')
    def validate_text_length(cls, v):
        if len(v.encode('utf-8')) > 10240:  # 10KB in bytes
            raise ValueError('Selected text must be at most 10KB')
        return v


class SelectionQueryResponse(BaseModel):
    question: str
    answer: str
    confidence: str = Field(..., pattern=r'^(high|medium|low)$')
    sufficient_information: bool


class ErrorResponse(BaseModel):
    status: str = "error"
    message: str
    error_code: Optional[str] = None