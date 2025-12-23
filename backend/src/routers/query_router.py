from fastapi import APIRouter, HTTPException
from typing import Optional
import logging

from ..routers.schemas import (
    QueryRequest, QueryResponse, 
    SelectionQueryRequest, SelectionQueryResponse,
    ErrorResponse
)
from ..services.query_service import query_service
from ..utils.logging_config import get_logger

logger = get_logger(__name__)

# Create the router for query endpoints
query_router = APIRouter(prefix="/query", tags=["query"])


@query_router.post("/", response_model=QueryResponse)
async def query_document(request: QueryRequest):
    """
    Query book content globally.
    Accepts user questions and returns answers based on ingested book content.
    """
    try:
        logger.info(f"Received query: {request.question[:50]}...")
        
        # Process the query using the query service
        result = await query_service.process_query(
            question=request.question,
            session_id=request.session_id
        )
        
        if result is None:
            raise HTTPException(status_code=500, detail="Query processing failed")
        
        # Log the result
        if result.get("answer"):
            logger.info("Query processed successfully")
        else:
            logger.error("Query processing returned no answer")
        
        return QueryResponse(**result)
        
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Unexpected error during querying: {str(e)}")
        raise HTTPException(
            status_code=500, 
            detail=f"Internal server error during querying: {str(e)}"
        )


@query_router.post("/selection", response_model=SelectionQueryResponse)
async def query_selection(request: SelectionQueryRequest):
    """
    Query using selected text only.
    Accepts user-selected text and question, bypasses vector store entirely.
    """
    try:
        logger.info(f"Received selection query for text excerpt: {request.selected_text[:50]}...")
        
        # Process the selection query using the query service
        result = await query_service.process_selection_query(
            selected_text=request.selected_text,
            question=request.question
        )
        
        if result is None:
            raise HTTPException(status_code=500, detail="Selection query processing failed")
        
        # Log the result
        if result.get("answer"):
            logger.info("Selection query processed successfully")
        else:
            logger.error("Selection query processing returned no answer")
        
        return SelectionQueryResponse(**result)
        
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Unexpected error during selection querying: {str(e)}")
        raise HTTPException(
            status_code=500, 
            detail=f"Internal server error during selection querying: {str(e)}"
        )


@query_router.get("/health")
async def query_health():
    """Health check endpoint for the query service"""
    return {"status": "healthy", "service": "query"}