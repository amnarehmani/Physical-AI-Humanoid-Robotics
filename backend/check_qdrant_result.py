import asyncio
from qdrant_client import QdrantClient
from qdrant_client.http import models
from src.config import settings


async def check_query_points_result():
    """Check the structure of results from query_points"""
    
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
    )
    
    # Try a simple query to see the result structure
    collection_name = "book_chunks"
    
    # Generate a simple embedding for testing
    from src.services.embedder_service import default_embedder
    query_embedding = default_embedder.generate_query_embedding("test query")
    
    results = client.query_points(
        collection_name=collection_name,
        query=query_embedding,
        limit=1,
        with_payload=True
    )
    
    print("Results type:", type(results))
    print("Results:", results)
    
    if results and len(results) > 0:
        first_result = results[0]
        print("First result type:", type(first_result))
        print("First result:", first_result)
        print("First result attributes:", dir(first_result))


if __name__ == "__main__":
    asyncio.run(check_query_points_result())