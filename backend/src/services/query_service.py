import time
import asyncio
from typing import Optional, Dict, Any, List
import logging
from uuid import UUID, uuid4

from ..config import settings
from ..utils.logging_config import get_logger
from .qdrant_service import qdrant_store
from .embedder_service import default_embedder
from .neon_metadata_service import neon_metadata_store
from ..models.document_model import DocumentStatus

logger = get_logger(__name__)


class QueryService:
    """
    Service class to coordinate the query process
    """
    
    def __init__(self):
        self.grounding_check_enabled = True
        self.hallucination_threshold = 0.7  # Confidence threshold for determining hallucination risk
    
    async def process_query(self, question: str, session_id: Optional[str] = None) -> Optional[Dict[str, Any]]:
        """
        Process a global query against the ingested documents
        
        Args:
            question: The user's question
            session_id: Optional session identifier for continuity
            
        Returns:
            Dictionary with the query response or None if failed
        """
        try:
            start_time = time.time()
            logger.info(f"Processing query: {question[:50]}...")
            
            # Generate embedding for the question
            query_embedding = default_embedder.generate_query_embedding(question)
            
            # Search for relevant chunks in Qdrant
            search_results = qdrant_store.search_vectors(
                query_embedding=query_embedding,
                limit=settings.MAX_RETRIEVAL_RESULTS
            )
            
            if not search_results:
                logger.info("No relevant chunks found for the query")
                return {
                    "question": question,
                    "answer": "I couldn't find any information in the book content that answers your question.",
                    "retrieved_chunks": [],
                    "confidence": "low"
                }
            
            # Assemble context from retrieved chunks
            context_chunks = []
            for result in search_results:
                # Retrieve full content of the chunk from metadata store
                chunk_details = await neon_metadata_store.get_chunks_by_document(
                    UUID(result["document_id"])
                )
                
                # Find the specific chunk that matches the result
                matching_chunk = None
                for chunk in chunk_details:
                    if chunk['chunk_id'] == result['chunk_id']:
                        matching_chunk = chunk
                        break
                
                if matching_chunk:
                    context_chunks.append({
                        "chunk_id": result["chunk_id"],
                        "content": matching_chunk.get('content_preview', result.get('content_preview', '')),
                        "source_document": result.get('source_document', 'Unknown'),
                        "section_title": result.get('section_title', 'Unknown')
                    })
            
            # Generate response using Cohere with the context
            context_text = "\n\n".join([chunk['content'] for chunk in context_chunks])
            answer = self._generate_answer_with_context(question, context_text)
            
            # Determine confidence based on number of retrieved chunks and relevance scores
            confidence = self._calculate_confidence(search_results)
            
            response_time = int((time.time() - start_time) * 1000)  # Convert to milliseconds
            
            # Log the query
            if session_id:
                await self._log_query(question, answer, context_chunks, session_id, response_time)
            
            logger.info(f"Query processed successfully in {response_time}ms")
            
            return {
                "question": question,
                "answer": answer,
                "retrieved_chunks": context_chunks,
                "confidence": confidence
            }
            
        except Exception as e:
            logger.error(f"Error during query processing: {str(e)}")
            return None
    
    async def process_selection_query(self, selected_text: str, question: str) -> Optional[Dict[str, Any]]:
        """
        Process a query based only on the selected text (bypassing vector store)
        
        Args:
            selected_text: The user-selected text to use as context
            question: The user's question about the selected text
            
        Returns:
            Dictionary with the query response or None if failed
        """
        try:
            start_time = time.time()
            logger.info(f"Processing selection query for text excerpt: {selected_text[:50]}...")
            
            # Generate response using only the selected text as context
            answer = self._generate_answer_with_context(question, selected_text)
            
            # Check if the answer is based only on the provided selected text
            sufficient_information = self._check_information_sufficiency(selected_text, answer)
            
            # Determine confidence
            confidence = "high" if sufficient_information else "low"
            
            response_time = int((time.time() - start_time) * 1000)  # Convert to milliseconds
            
            logger.info(f"Selection query processed successfully in {response_time}ms")
            
            return {
                "question": question,
                "answer": answer,
                "confidence": confidence,
                "sufficient_information": sufficient_information
            }
            
        except Exception as e:
            logger.error(f"Error during selection query processing: {str(e)}")
            return None
    
    def _generate_answer_with_context(self, question: str, context: str) -> str:
        """
        Generate an answer using the provided context
        """
        try:
            # Construct a prompt that explicitly uses the context to answer the question
            message = (
                f"Based strictly on the following context, answer the question. "
                f"If the context does not contain information to answer the question, "
                f"say so explicitly.\n\n"
                f"Context: {context}\n\n"
                f"Question: {question}\n\n"
                f"Answer (based only on the provided context):"
            )

            # Use Cohere to generate the response with the new API
            try:
                response = default_embedder.client.chat(
                    model='command-r',  # Using the new command-r model
                    message=message,
                    temperature=0.1,  # Low temperature for consistency and to reduce hallucinations
                    max_tokens=500
                )

                answer = response.text.strip() if response.text else ""
            except Exception as e:
                logger.error(f"Cohere API error: {str(e)}")
                # Fallback response using the context directly
                answer = f"Based on the provided context: {context[:500]}..."

            # Validate that the response is grounded in the context
            if not self._is_response_grounded(answer, context):
                logger.warning("Generated response may not be fully grounded in context")
                return "I cannot answer this question as the provided context does not contain sufficient information."

            return answer
            
        except Exception as e:
            logger.error(f"Error generating answer with context: {str(e)}")
            return "There was an error generating the answer. Please try again."
    
    def _is_response_grounded(self, answer: str, context: str) -> bool:
        """
        Check if the answer is grounded in the provided context
        """
        # This is a basic check - in a production system, you'd want more sophisticated grounding verification
        answer_lower = answer.lower()
        context_lower = context.lower()
        
        # Check if answer contains key phrases from context
        context_sentences = [s.strip() for s in context_lower.split('.') if len(s.strip()) > 10]
        for sentence in context_sentences:
            if len(sentence) > 20 and sentence in answer_lower:
                return True
        
        # If no direct matches found, check for semantic similarity in key terms
        context_words = set(context_lower.split())
        answer_words = set(answer_lower.split())
        
        # If the intersection of words is significant, consider it grounded
        if len(context_words & answer_words) / len(answer_words) > 0.3:
            return True
        
        return False
    
    def _calculate_confidence(self, search_results: List[Dict[str, Any]]) -> str:
        """
        Calculate confidence level based on search results
        """
        if not search_results:
            return "low"
        
        # Calculate average score of retrieved chunks
        avg_score = sum([result.get('score', 0) for result in search_results]) / len(search_results)
        
        # Determine confidence based on score thresholds
        if avg_score > 0.7:
            return "high"
        elif avg_score > 0.4:
            return "medium"
        else:
            return "low"
    
    def _check_information_sufficiency(self, selected_text: str, answer: str) -> bool:
        """
        Check if the selected text contains sufficient information to answer the question
        """
        # A basic check: if the answer is "I don't know" or similar phrases, 
        # it means the information wasn't sufficient
        answer_lower = answer.lower()
        
        insufficient_indicators = [
            "i don't know", "i do not know", "not mentioned", "not specified", 
            "not provided", "cannot determine", "insufficient information"
        ]
        
        for indicator in insufficient_indicators:
            if indicator in answer_lower:
                return False
        
        return True
    
    async def _log_query(self, question: str, answer: str, retrieved_chunks: List[Dict], 
                         session_id: str, response_time: int):
        """
        Log the query for analytics and debugging
        """
        try:
            log_data = {
                "session_id": session_id,
                "query_text": question,
                "response_text": answer,
                "retrieved_chunks": retrieved_chunks,
                "response_time_ms": response_time
            }
            
            await neon_metadata_store.create_query_log(log_data)
        except Exception as e:
            logger.error(f"Error logging query: {str(e)}")


# Global instance of the QueryService
query_service = QueryService()