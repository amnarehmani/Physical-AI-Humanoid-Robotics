from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional
from uuid import UUID


class VectorStoreInterface(ABC):
    """
    Abstract interface for vector storage operations
    """
    
    @abstractmethod
    def upsert_vectors(
        self, 
        chunk_ids: List[str], 
        embeddings: List[List[float]], 
        payloads: List[Dict[str, Any]]
    ) -> bool:
        """
        Upsert vectors with their embeddings and payloads
        """
        pass
    
    @abstractmethod
    def search_vectors(
        self, 
        query_embedding: List[float], 
        limit: int = 10,
        document_id_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors based on the query embedding
        """
        pass
    
    @abstractmethod
    def delete_document_chunks(self, document_id: str) -> bool:
        """
        Delete all chunks associated with a specific document
        """
        pass
    
    @abstractmethod
    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID
        """
        pass


class MetadataStoreInterface(ABC):
    """
    Abstract interface for metadata storage operations
    """
    
    @abstractmethod
    async def create_document(self, document_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Create a new document record in the metadata store
        """
        pass
    
    @abstractmethod
    async def update_document_status(self, document_id: UUID, status: str) -> bool:
        """
        Update the status of a document
        """
        pass
    
    @abstractmethod
    async def get_document(self, document_id: UUID) -> Optional[Dict[str, Any]]:
        """
        Retrieve a document by its ID
        """
        pass
    
    @abstractmethod
    async def create_chunk(self, chunk_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Create a new chunk record in the metadata store
        """
        pass
    
    @abstractmethod
    async def get_chunks_by_document(self, document_id: UUID) -> List[Dict[str, Any]]:
        """
        Retrieve all chunks associated with a specific document
        """
        pass
    
    @abstractmethod
    async def create_query_log(self, log_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Create a query log entry
        """
        pass
    
    @abstractmethod
    async def get_document_by_source_path(self, source_path: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a document by its source path
        """
        pass