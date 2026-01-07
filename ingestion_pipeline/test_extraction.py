"""
Utility script to test content extraction from the deployed website
"""
import asyncio
import os
from ingestion_pipeline.ingest import WebsiteIngestor


async def test_content_extraction():
    """Test content extraction from a sample URL"""
    # Use the deployed website URL
    base_url = "https://hackathon1-book-ragchatbot.vercel.app"
    
    # Mock API keys for testing (won't actually make API calls in this test)
    ingestor = WebsiteIngestor(
        base_url=base_url,
        qdrant_url="test",
        qdrant_api_key="test",
        openai_api_key="test"
    )
    
    # Test with a specific URL from the Docusaurus site
    test_url = f"{base_url}/docs/modules/ros2/intro"
    print(f"Testing content extraction from: {test_url}")
    
    content_data = await ingestor.extract_content(test_url)
    
    if content_data:
        print(f"Title: {content_data['title']}")
        print(f"Content length: {len(content_data['content'])} characters")
        print(f"Preview: {content_data['content'][:500]}...")
    else:
        print("Failed to extract content")
    
    # Test chunking
    if content_data and content_data.get('content'):
        chunks = ingestor.chunk_content(content_data['content'])
        print(f"\nContent was chunked into {len(chunks)} pieces")
        for i, chunk in enumerate(chunks[:3]):  # Show first 3 chunks
            print(f"Chunk {i+1}: {len(chunk)} characters - {chunk[:100]}...")


if __name__ == "__main__":
    asyncio.run(test_content_extraction())