import asyncio
import json
from src.services.query_service import query_service


async def test_query():
    """Test the query process with sample questions"""
    
    print("Starting query test...")
    
    # Test questions about the sample book content
    test_questions = [
        "What is Physical AI Robotics?",
        "What are the core technologies in physical AI robotics?",
        "What are the applications of physical AI robotics?",
        "What challenges does physical AI robotics face?"
    ]
    
    for i, question in enumerate(test_questions, 1):
        print(f"\n--- Test Query {i} ---")
        print(f"Question: {question}")
        
        # Process the query
        result = await query_service.process_query(
            question=question,
            session_id=f"test_session_{i}"
        )
        
        print("Query result:")
        if result:
            print(json.dumps(result, indent=2, default=str))
            
            if result.get("answer") and "I couldn't find any information" not in result.get("answer", ""):
                print("SUCCESS: Query successful - found relevant information")
            else:
                print("FAILED: Query failed - no relevant information found")
        else:
            print("FAILED: Query failed - no result returned")


if __name__ == "__main__":
    asyncio.run(test_query())