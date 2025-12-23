from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from uuid import UUID
import logging

from ..config import settings
from ..utils.logging_config import get_logger

logger = get_logger(__name__)


class QdrantStore:
    """
    Service class to handle vector storage operations with Qdrant
    """
    
    def __init__(self):
        # Initialize Qdrant client
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            # For production, consider using a cluster setup
        )
        
        # Collection name for book chunks
        self.collection_name = "book_chunks"
        
        # Create the collection if it doesn't exist
        self._initialize_collection()
    
    def _initialize_collection(self):
        """Initialize the Qdrant collection with the required schema"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)
            
            if not collection_exists:
                # Create collection with specified vector size for Cohere embeddings
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
                )
                
                # Create payload index for document_id to improve search performance
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="document_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )
                
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection already exists: {self.collection_name}")
                
        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {str(e)}")
            raise
    
    def upsert_vectors(
        self, 
        chunk_ids: List[str], 
        embeddings: List[List[float]], 
        payloads: List[Dict[str, Any]]
    ) -> bool:
        """
        Upsert vectors with their embeddings and payloads to Qdrant
        
        Args:
            chunk_ids: List of unique IDs for the chunks
            embeddings: List of embedding vectors
            payloads: List of metadata payloads for each vector
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Create points to upsert
            points = [
                models.PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload=payload
                )
                for chunk_id, embedding, payload in zip(chunk_ids, embeddings, payloads)
            ]
            
            # Upsert the points
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            
            logger.info(f"Successfully upserted {len(points)} vectors to Qdrant")
            return True
            
        except Exception as e:
            logger.error(f"Error upserting vectors to Qdrant: {str(e)}")
            return False
    
    def search_vectors(
        self, 
        query_embedding: List[float], 
        limit: int = 10,
        document_id_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant based on the query embedding
        
        Args:
            query_embedding: The embedding to search for similarities
            limit: Maximum number of results to return
            document_id_filter: Optional filter to only search within a specific document
            
        Returns:
            List of matching vectors with their payloads
        """
        try:
            # Prepare filters if document_id is specified
            search_filter = None
            if document_id_filter:
                search_filter = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="document_id",
                            match=models.MatchValue(value=document_id_filter)
                        )
                    ]
                )
            
            # Perform the search using the query_points method
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                query_filter=search_filter,
                limit=limit,
                with_payload=True
            )

            # Extract and format the results
            # The query_points method returns a QueryResponse object with points
            formatted_results = []
            for result in results.points:
                formatted_results.append({
                    "chunk_id": str(result.id),
                    "document_id": result.payload.get("document_id"),
                    "section_title": result.payload.get("section_title"),
                    "source_path": result.payload.get("source_path"),
                    "content_preview": result.payload.get("content_preview"),
                    "chunk_order": result.payload.get("chunk_order"),
                    "metadata": result.payload.get("metadata", {}),
                    "score": result.score
                })

            logger.info(f"Found {len(formatted_results)} similar vectors in Qdrant")
            return formatted_results
            
        except Exception as e:
            logger.error(f"Error searching vectors in Qdrant: {str(e)}")
            return []
    
    def delete_document_chunks(self, document_id: str) -> bool:
        """
        Delete all chunks associated with a specific document
        
        Args:
            document_id: The ID of the document to delete chunks for
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Create filter for the specific document
            filter_condition = models.Filter(
                must=[
                    models.FieldCondition(
                        key="document_id",
                        match=models.MatchValue(value=document_id)
                    )
                ]
            )
            
            # Delete points matching the filter
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(
                    filter=filter_condition
                )
            )
            
            logger.info(f"Deleted all chunks for document {document_id}")
            return True
            
        except Exception as e:
            logger.error(f"Error deleting document chunks from Qdrant: {str(e)}")
            return False
    
    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID
        
        Args:
            chunk_id: The ID of the chunk to retrieve
            
        Returns:
            The chunk data if found, None otherwise
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id],
                with_payload=True
            )
            
            if records and len(records) > 0:
                record = records[0]
                return {
                    "chunk_id": record.id,
                    "document_id": record.payload.get("document_id"),
                    "section_title": record.payload.get("section_title"),
                    "source_path": record.payload.get("source_path"),
                    "content_preview": record.payload.get("content_preview"),
                    "chunk_order": record.payload.get("chunk_order"),
                    "metadata": record.payload.get("metadata", {})
                }
            else:
                return None
            
        except Exception as e:
            logger.error(f"Error retrieving chunk {chunk_id} from Qdrant: {str(e)}")
            return None


# Global instance of QdrantStore
qdrant_store = QdrantStore()