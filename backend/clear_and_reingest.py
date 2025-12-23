import asyncio
import asyncpg
from uuid import uuid4
from src.services.qdrant_service import qdrant_store
from src.services.ingestion_service import ingestion_service
from src.services.neon_metadata_service import neon_metadata_store


async def clear_and_reingest():
    """Clear existing data and re-ingest with correct embeddings"""
    
    print("Clearing existing data...")
    
    # Clear Qdrant collection
    print("Clearing Qdrant collection...")
    try:
        qdrant_store.client.delete(
            collection_name=qdrant_store.collection_name,
            points_selector=qdrant_store.client.get_collection(qdrant_store.collection_name).points_count
        )
        print("Qdrant collection cleared")
    except Exception as e:
        print(f"Error clearing Qdrant collection: {str(e)}")
        # If we can't delete all points, try recreating the collection
        try:
            qdrant_store.client.delete_collection(qdrant_store.collection_name)
            qdrant_store._initialize_collection()  # Reinitialize with correct vector size
            print("Qdrant collection recreated")
        except Exception as e2:
            print(f"Error recreating Qdrant collection: {str(e2)}")
    
    # Clear database records
    print("Clearing database records...")
    try:
        conn = await neon_metadata_store.get_connection()
        
        # Clear chunks first (due to foreign key constraint)
        await conn.execute("DELETE FROM chunks")
        # Then clear documents
        await conn.execute("DELETE FROM documents")
        # And query logs
        await conn.execute("DELETE FROM query_logs")
        
        await conn.close()
        print("Database records cleared")
    except Exception as e:
        print(f"Error clearing database records: {str(e)}")
    
    # Now re-ingest with correct embeddings
    print("Re-ingesting with correct embeddings...")
    
    # Read the sample book content with a slight modification to force re-ingestion
    import pathlib
    sample_content_path = pathlib.Path("../sample_book_content.md")
    if not sample_content_path.exists():
        print(f"Sample content file not found: {sample_content_path}")
        return
    
    content = sample_content_path.read_text(encoding='utf-8')
    # Add a timestamp to force re-ingestion
    modified_content = content + f"\n\n<!-- Re-ingestion timestamp: {uuid4()} -->"
    
    print(f"Content length: {len(modified_content)} characters")
    
    # Test the ingestion
    result = await ingestion_service.ingest_document(
        content=modified_content,
        source_path="/docs/sample_book.md",
        title="Sample Book on Physical AI Robotics"
    )
    
    import json
    print("\nIngestion result:")
    print(json.dumps(result, indent=2, default=str))
    
    if result and result.get("status") == "success":
        print(f"\nSUCCESS: Re-ingestion successful!")
        print(f"Document ID: {result.get('document_id')}")
        print(f"Chunks processed: {result.get('chunks_processed')}")
    else:
        print(f"\nFAILED: Re-ingestion failed!")
        print(f"Message: {result.get('message') if result else 'No result returned'}")


if __name__ == "__main__":
    asyncio.run(clear_and_reingest())