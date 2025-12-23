import asyncio
import hashlib
from typing import Optional, Dict, Any, List
import logging
from uuid import UUID, uuid4

from ..config import settings
from ..utils.logging_config import get_logger
from .chunker_service import default_chunker
from .embedder_service import default_embedder
from .qdrant_service import qdrant_store
from .neon_metadata_service import neon_metadata_store
from .content_validator import content_validator
from ..models.document_model import DocumentStatus

logger = get_logger(__name__)


class IngestionService:
    """
    Service class to coordinate the ingestion process
    """
    
    def __init__(self):
        self.rate_limit_semaphore = asyncio.Semaphore(settings.MAX_CONCURRENT_INGEST)
    
    async def ingest_document(
        self,
        content: str,
        source_path: str,
        title: str
    ) -> Optional[Dict[str, Any]]:
        """
        Main method to ingest a document with all steps

        Args:
            content: The document content to ingest
            source_path: The source path of the document
            title: The title of the document

        Returns:
            Dictionary with ingestion result or None if failed
        """
        # Acquire semaphore to respect rate limiting
        async with self.rate_limit_semaphore:
            try:
                logger.info(f"Starting ingestion for document: {title}")

                # Validate content format and size
                if not content_validator.validate_format(content):
                    logger.error("Content format validation failed")
                    return {
                        "status": "error",
                        "message": "Invalid content format",
                        "document_id": None
                    }

                if not content_validator.validate_size(content):
                    logger.error("Content size validation failed")
                    return {
                        "status": "error",
                        "message": "Content exceeds maximum allowed size",
                        "document_id": None
                    }

                # Sanitize the content
                sanitized_content = content_validator.sanitize_content(content)

                # Calculate checksum for change detection
                checksum = hashlib.sha256(sanitized_content.encode()).hexdigest()

                # Check if document already exists with same checksum
                existing_doc = await neon_metadata_store.get_document_by_source_path(source_path)
                if existing_doc and existing_doc['checksum'] == checksum:
                    logger.info(f"Document {source_path} already exists with same content, skipping ingestion")
                    return {
                        "status": "success",
                        "message": "Document already exists with same content",
                        "document_id": existing_doc['id'],
                        "chunks_processed": existing_doc['chunk_count']
                    }

                # Create document record in metadata store
                document_data = {
                    "title": title,
                    "source_path": source_path,
                    "content": sanitized_content,  # Use sanitized content for checksum
                    "status": DocumentStatus.PROCESSING.value,
                    "chunk_count": 0
                }

                document_record = await neon_metadata_store.create_document(document_data)
                if not document_record:
                    logger.error("Failed to create document record")
                    return None

                document_id = document_record['id']
                logger.info(f"Created document record with ID: {document_id}")

                # Update document status to processing
                await neon_metadata_store.update_document_status(
                    document_id,
                    DocumentStatus.PROCESSING.value
                )

                # Chunk the sanitized document
                logger.info("Starting document chunking...")
                chunks = default_chunker.chunk_document(sanitized_content, source_path)
                logger.info(f"Document chunked into {len(chunks)} chunks")

                if not chunks:
                    logger.error("No chunks were created from the document")
                    await neon_metadata_store.update_document_status(
                        document_id,
                        DocumentStatus.FAILED.value
                    )
                    return {
                        "status": "error",
                        "message": "No content could be extracted from the document",
                        "document_id": document_id
                    }

                # Generate embeddings for all chunks at once
                logger.info("Generating embeddings for chunks...")
                chunk_texts = [chunk['content'] for chunk in chunks]
                embeddings = default_embedder.generate_embeddings(chunk_texts)

                if len(embeddings) != len(chunks):
                    logger.error("Mismatch between number of chunks and embeddings")
                    await neon_metadata_store.update_document_status(
                        document_id,
                        DocumentStatus.FAILED.value
                    )
                    return {
                        "status": "error",
                        "message": "Embedding generation failed",
                        "document_id": document_id
                    }

                # Prepare data for vector storage
                chunk_ids = [chunk['chunk_id'] for chunk in chunks]
                payloads = []

                for i, chunk in enumerate(chunks):
                    payload = {
                        "document_id": str(document_id),
                        "chunk_id": chunk['chunk_id'],
                        "section_title": chunk['section_title'],
                        "source_path": chunk['source_path'],
                        "content_preview": chunk['content'][:500],
                        "chunk_order": chunk['chunk_order'],
                        "metadata": {
                            "paragraph_count": chunk['content'].count('\n\n') + 1,
                            "word_count": len(chunk['content'].split()),
                            "language": "en"  # Could be detected
                        }
                    }
                    payloads.append(payload)

                # Store embeddings in Qdrant
                logger.info("Storing embeddings in Qdrant...")
                success = qdrant_store.upsert_vectors(chunk_ids, embeddings, payloads)

                if not success:
                    logger.error("Failed to store embeddings in Qdrant")
                    await neon_metadata_store.update_document_status(
                        document_id,
                        DocumentStatus.FAILED.value
                    )
                    return {
                        "status": "error",
                        "message": "Failed to store embeddings in vector database",
                        "document_id": document_id
                    }

                # Store chunk metadata in Neon
                logger.info("Storing chunk metadata in Neon...")
                for i, chunk in enumerate(chunks):
                    chunk_data = {
                        "document_id": document_id,
                        "chunk_order": chunk['chunk_order'],
                        "chunk_id": chunk['chunk_id'],
                        "content": chunk['content'],
                        "section_title": chunk['section_title'],
                        "metadata": payloads[i]['metadata']
                    }

                    chunk_record = await neon_metadata_store.create_chunk(chunk_data)
                    if not chunk_record:
                        logger.error(f"Failed to create chunk record for chunk {i}")
                        await neon_metadata_store.update_document_status(
                            document_id,
                            DocumentStatus.FAILED.value
                        )
                        return {
                            "status": "error",
                            "message": f"Failed to store chunk metadata: {i}",
                            "document_id": document_id
                        }

                # Update document status to completed and set chunk count
                await neon_metadata_store.update_document_status(
                    document_id,
                    DocumentStatus.COMPLETED.value
                )

                # Update document's chunk count
                await self._update_document_chunk_count(document_id, len(chunks))

                logger.info(f"Ingestion completed successfully for document {document_id}")

                return {
                    "status": "success",
                    "message": f"Successfully ingested document with {len(chunks)} chunks",
                    "document_id": document_id,
                    "chunks_processed": len(chunks)
                }

            except Exception as e:
                logger.error(f"Error during document ingestion: {str(e)}")

                # Try to update document status to failed
                if 'document_id' in locals():
                    try:
                        await neon_metadata_store.update_document_status(
                            document_id,
                            DocumentStatus.FAILED.value
                        )
                    except:
                        pass  # Swallow errors during error handling

                return {
                    "status": "error",
                    "message": f"Ingestion failed: {str(e)}",
                    "document_id": document_id if 'document_id' in locals() else None
                }
    
    async def _update_document_chunk_count(self, document_id: UUID, chunk_count: int):
        """
        Update the chunk count for a document
        """
        conn = None
        try:
            conn = await neon_metadata_store.get_connection()
            
            query = """
                UPDATE documents
                SET chunk_count = $1, updated_at = NOW()
                WHERE id = $2
            """
            
            await conn.execute(query, chunk_count, document_id)
            logger.info(f"Updated chunk count to {chunk_count} for document {document_id}")
            
        except Exception as e:
            logger.error(f"Error updating document chunk count: {str(e)}")
        finally:
            if conn:
                await conn.close()


# Global instance of the IngestionService
ingestion_service = IngestionService()