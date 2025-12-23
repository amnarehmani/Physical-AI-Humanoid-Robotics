import asyncio
import json
from pathlib import Path

from src.services.ingestion_service import ingestion_service


async def test_ingestion():
    """Test the ingestion process with sample content"""
    
    # Read the sample book content
    sample_content_path = Path("../sample_book_content.md")
    if not sample_content_path.exists():
        print(f"Sample content file not found: {sample_content_path}")
        return
    
    content = sample_content_path.read_text(encoding='utf-8')
    
    print("Starting ingestion test...")
    print(f"Content length: {len(content)} characters")
    
    # Test the ingestion
    result = await ingestion_service.ingest_document(
        content=content,
        source_path="/docs/sample_book.md",
        title="Sample Book on Physical AI Robotics"
    )
    
    print("\nIngestion result:")
    print(json.dumps(result, indent=2, default=str))
    
    if result and result.get("status") == "success":
        print(f"\nSUCCESS: Ingestion successful!")
        print(f"Document ID: {result.get('document_id')}")
        print(f"Chunks processed: {result.get('chunks_processed')}")
    else:
        print(f"\nFAILED: Ingestion failed!")
        print(f"Message: {result.get('message') if result else 'No result returned'}")


if __name__ == "__main__":
    asyncio.run(test_ingestion())