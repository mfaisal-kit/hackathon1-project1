"""
Script to check if Qdrant database has been properly populated
"""
import os
import asyncio
from qdrant_client import AsyncQdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

async def check_qdrant_database():
    # Initialize Qdrant client
    qdrant_client = AsyncQdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    
    try:
        # Get collection information
        collection_name = "book_content"
        collection_info = await qdrant_client.get_collection(collection_name)
        
        print(f"Collection '{collection_name}' exists")
        print(f"Points count: {collection_info.points_count}")
        
        # Try to get a few sample points
        if collection_info.points_count > 0:
            # Get sample points
            sample_points = await qdrant_client.scroll(
                collection_name=collection_name,
                limit=3,
                with_payload=True,
                with_vectors=False
            )
            
            print("\nSample points from the database:")
            for i, point in enumerate(sample_points[0]):
                print(f"\nPoint {i+1}:")
                print(f"  ID: {point.id}")
                print(f"  URL: {point.payload.get('url', 'N/A')}")
                print(f"  Title: {point.payload.get('title', 'N/A')}")
                print(f"  Content preview: {point.payload.get('content', '')[:100]}...")
        else:
            print("\nNo points found in the database. You need to run the ingestion pipeline first.")
            
    except Exception as e:
        print(f"Error accessing Qdrant: {str(e)}")
        print("Make sure your QDRANT_URL and QDRANT_API_KEY are set correctly in your .env file")

if __name__ == "__main__":
    asyncio.run(check_qdrant_database())