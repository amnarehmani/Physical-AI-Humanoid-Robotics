import asyncpg
import logging
import json
from typing import List, Dict, Any, Optional
from uuid import UUID, uuid4
from datetime import datetime
import hashlib

from .interfaces import MetadataStoreInterface
from ..config import settings
from ..utils.logging_config import get_logger

logger = get_logger(__name__)


class NeonMetadataStore(MetadataStoreInterface):
    """
    Implementation of MetadataStoreInterface using Neon Postgres
    """
    
    def __init__(self):
        self.connection_string = settings.NEON_DATABASE_URL
        if not self.connection_string:
            raise ValueError("NEON_DATABASE_URL environment variable is required")
    
    async def get_connection(self):
        """Get a connection to the Neon database"""
        try:
            conn = await asyncpg.connect(self.connection_string)
            return conn
        except Exception as e:
            logger.error(f"Failed to connect to Neon database: {str(e)}")
            raise
    
    async def create_document(self, document_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Create a new document record in the metadata store
        """
        conn = None
        try:
            conn = await self.get_connection()
            
            # Calculate checksum if not provided
            if 'checksum' not in document_data:
                content = document_data.get('content', '')
                document_data['checksum'] = hashlib.sha256(content.encode()).hexdigest()

            # Check if a document with the same source path already exists
            existing_doc = await self.get_document_by_source_path(document_data['source_path'])

            if existing_doc:
                # If document exists, increment the version
                new_version = existing_doc['version'] + 1
                doc_id = existing_doc['id']  # Use the same document ID for versioning

                # Update the existing document with new version
                query = """
                    UPDATE documents
                    SET title = $1,
                        checksum = $2,
                        version = $3,
                        status = $4,
                        chunk_count = 0,
                        updated_at = NOW()
                    WHERE id = $5
                    RETURNING id, title, source_path, checksum, version,
                             created_at, updated_at, status, chunk_count
                """

                record = await conn.fetchrow(
                    query,
                    document_data['title'],
                    document_data['checksum'],
                    new_version,
                    document_data.get('status', 'pending'),
                    doc_id
                )
            else:
                # Generate document ID if not provided (for new documents)
                doc_id = document_data.get('id', uuid4())

                # Insert a new document
                query = """
                    INSERT INTO documents (
                        id, title, source_path, checksum, version,
                        status, chunk_count
                    )
                    VALUES ($1, $2, $3, $4, $5, $6, $7)
                    RETURNING id, title, source_path, checksum, version,
                             created_at, updated_at, status, chunk_count
                """

                record = await conn.fetchrow(
                    query,
                    doc_id,
                    document_data['title'],
                    document_data['source_path'],
                    document_data['checksum'],
                    document_data.get('version', 1),
                    document_data.get('status', 'pending'),
                    document_data.get('chunk_count', 0)
                )
            
            if record:
                result = dict(record)
                logger.info(f"Created document with ID: {result['id']}")
                return result
            else:
                logger.error("Failed to create document")
                return None
                
        except Exception as e:
            logger.error(f"Error creating document: {str(e)}")
            raise
        finally:
            if conn:
                await conn.close()
    
    async def update_document_status(self, document_id: UUID, status: str) -> bool:
        """
        Update the status of a document
        """
        conn = None
        try:
            conn = await self.get_connection()
            
            query = """
                UPDATE documents
                SET status = $1, updated_at = NOW()
                WHERE id = $2
                RETURNING id
            """
            
            record = await conn.fetchrow(query, status, document_id)
            
            if record:
                logger.info(f"Updated document {document_id} status to {status}")
                return True
            else:
                logger.warning(f"No document found with ID: {document_id}")
                return False
                
        except Exception as e:
            logger.error(f"Error updating document status: {str(e)}")
            raise
        finally:
            if conn:
                await conn.close()
    
    async def get_document(self, document_id: UUID) -> Optional[Dict[str, Any]]:
        """
        Retrieve a document by its ID
        """
        conn = None
        try:
            conn = await self.get_connection()
            
            query = """
                SELECT id, title, source_path, checksum, version, 
                       created_at, updated_at, status, chunk_count
                FROM documents
                WHERE id = $1
            """
            
            record = await conn.fetchrow(query, document_id)
            
            if record:
                return dict(record)
            else:
                logger.info(f"No document found with ID: {document_id}")
                return None
                
        except Exception as e:
            logger.error(f"Error retrieving document: {str(e)}")
            raise
        finally:
            if conn:
                await conn.close()
    
    async def create_chunk(self, chunk_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Create a new chunk record in the metadata store
        """
        conn = None
        try:
            conn = await self.get_connection()
            
            # Generate chunk ID if not provided
            chunk_id = chunk_data.get('id', uuid4())
            
            query = """
                INSERT INTO chunks (
                    id, document_id, chunk_order, chunk_id,
                    content_preview, section_title, metadata
                )
                VALUES ($1, $2, $3, $4, $5, $6, $7::jsonb)
                RETURNING id, document_id, chunk_order, chunk_id,
                         content_preview, section_title, metadata, created_at
            """
            
            content_preview = chunk_data.get('content', '')[:500]  # First 500 chars
            
            record = await conn.fetchrow(
                query,
                chunk_id,
                chunk_data['document_id'],
                chunk_data['chunk_order'],
                chunk_data['chunk_id'],
                content_preview,
                chunk_data.get('section_title'),
                json.dumps(chunk_data.get('metadata', {}))
            )
            
            if record:
                result = dict(record)
                logger.info(f"Created chunk with ID: {result['id']}")
                return result
            else:
                logger.error("Failed to create chunk")
                return None
                
        except Exception as e:
            logger.error(f"Error creating chunk: {str(e)}")
            raise
        finally:
            if conn:
                await conn.close()
    
    async def get_chunks_by_document(self, document_id: UUID) -> List[Dict[str, Any]]:
        """
        Retrieve all chunks associated with a specific document
        """
        conn = None
        try:
            conn = await self.get_connection()
            
            query = """
                SELECT id, document_id, chunk_order, chunk_id, 
                       content_preview, section_title, metadata, created_at
                FROM chunks
                WHERE document_id = $1
                ORDER BY chunk_order
            """
            
            records = await conn.fetch(query, document_id)
            
            chunks = [dict(record) for record in records]
            logger.info(f"Retrieved {len(chunks)} chunks for document {document_id}")
            return chunks
                
        except Exception as e:
            logger.error(f"Error retrieving chunks: {str(e)}")
            raise
        finally:
            if conn:
                await conn.close()
    
    async def create_query_log(self, log_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Create a query log entry
        """
        conn = None
        try:
            conn = await self.get_connection()
            
            # Generate log ID if not provided
            log_id = log_data.get('id', uuid4())
            
            query = """
                INSERT INTO query_logs (
                    id, session_id, query_text, response_text, 
                    retrieved_chunks, response_time_ms
                )
                VALUES ($1, $2, $3, $4, $5, $6)
                RETURNING id, session_id, query_text, response_text, 
                         retrieved_chunks, timestamp, response_time_ms
            """
            
            import json
            record = await conn.fetchrow(
                query,
                log_id,
                log_data.get('session_id'),
                log_data['query_text'],
                log_data['response_text'],
                json.dumps(log_data.get('retrieved_chunks', [])),
                log_data.get('response_time_ms')
            )
            
            if record:
                result = dict(record)
                logger.info(f"Created query log with ID: {result['id']}")
                return result
            else:
                logger.error("Failed to create query log")
                return None
                
        except Exception as e:
            logger.error(f"Error creating query log: {str(e)}")
            raise
        finally:
            if conn:
                await conn.close()
    
    async def get_document_by_source_path(self, source_path: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a document by its source path
        """
        conn = None
        try:
            conn = await self.get_connection()
            
            query = """
                SELECT id, title, source_path, checksum, version, 
                       created_at, updated_at, status, chunk_count
                FROM documents
                WHERE source_path = $1
                ORDER BY version DESC
                LIMIT 1
            """
            
            record = await conn.fetchrow(query, source_path)
            
            if record:
                return dict(record)
            else:
                logger.info(f"No document found with source path: {source_path}")
                return None
                
        except Exception as e:
            logger.error(f"Error retrieving document by source path: {str(e)}")
            raise
        finally:
            if conn:
                await conn.close()


# Global instance of the NeonMetadataStore
neon_metadata_store = NeonMetadataStore()