from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks
from typing import Optional
import logging

from ..routers.schemas import IngestRequest, IngestResponse
from ..services.ingestion_service import ingestion_service
from ..utils.logging_config import get_logger

logger = get_logger(__name__)

# Create the router for ingest endpoints
ingest_router = APIRouter(prefix="/ingest", tags=["ingest"])


@ingest_router.post("/", response_model=IngestResponse)
async def ingest_document(request: IngestRequest):
    """
    Ingest book content for RAG processing.
    Accepts Markdown/MDX book content, processes it into chunks, and stores embeddings.
    """
    try:
        logger.info(f"Received ingestion request for: {request.source_path}")
        
        # Call the ingestion service
        result = await ingestion_service.ingest_document(
            content=request.content,
            source_path=request.source_path,
            title=request.title
        )
        
        if result is None:
            raise HTTPException(status_code=500, detail="Ingestion service failed to process request")
        
        # Log the result
        if result["status"] == "success":
            logger.info(f"Ingestion completed successfully: {result['message']}")
        else:
            logger.error(f"Ingestion failed: {result['message']}")
        
        return IngestResponse(**result)
        
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Unexpected error during ingestion: {str(e)}")
        raise HTTPException(
            status_code=500, 
            detail=f"Internal server error during ingestion: {str(e)}"
        )


@ingest_router.get("/health")
async def ingest_health():
    """Health check endpoint for the ingestion service"""
    return {"status": "healthy", "service": "ingestion"}