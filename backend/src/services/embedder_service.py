import cohere
from typing import List
import logging

from ..config import settings
from ..utils.logging_config import get_logger

logger = get_logger(__name__)


class Embedder:
    """
    Service class to handle text embedding using Cohere
    """
    
    def __init__(self):
        # Initialize Cohere client
        self.client = cohere.Client(settings.COHERE_API_KEY)
        self.model = "embed-english-v3.0"  # Updated model
    
    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of text chunks
        
        Args:
            texts: List of text chunks to embed
            
        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        try:
            # Cohere's embed function can handle multiple texts in one call
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document",  # Using search_document for content chunks
                embedding_types=["float"]  # Specify embedding type for v3 model
            )

            # Extract embeddings from the response
            embeddings = [embedding for embedding in response.embeddings.float]
            
            logger.info(f"Successfully generated embeddings for {len(texts)} text chunks")
            return embeddings
            
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise
    
    def generate_query_embedding(self, query: str) -> List[float]:
        """
        Generate embedding for a query text
        
        Args:
            query: Query text to embed
            
        Returns:
            Embedding vector (list of floats)
        """
        try:
            response = self.client.embed(
                texts=[query],
                model=self.model,
                input_type="search_query",  # Using search_query for user queries
                embedding_types=["float"]  # Specify embedding type for v3 model
            )

            # Extract the first (and only) embedding from the response
            embedding = response.embeddings.float[0] if response.embeddings.float else []
            
            logger.info("Successfully generated query embedding")
            return embedding
            
        except Exception as e:
            logger.error(f"Error generating query embedding: {str(e)}")
            raise


# Global instance of the embedder
default_embedder = Embedder()