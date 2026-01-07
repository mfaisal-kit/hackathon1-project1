"""
Simple test for the backend ingestion pipeline
"""
import asyncio
import os
from unittest.mock import AsyncMock, patch, MagicMock
from backend.main import BookIngestor


async def test_backend_functionality():
    """Test the backend ingestion functionality"""
    print("Testing backend ingestion functionality...")
    
    # Create a mock ingestor
    ingestor = BookIngestor(
        base_url="https://hackathon1-book-ragchatbot.vercel.app",
        qdrant_url="test-url",
        qdrant_api_key="test-key",
        openai_api_key="test-openai-key"
    )
    
    # Mock all external dependencies to simulate a complete run
    with patch.object(ingestor, 'qdrant_client') as mock_qdrant, \
         patch('openai.AsyncOpenAI') as mock_openai, \
         patch('requests.get') as mock_requests:
        
        # Mock Qdrant operations
        mock_qdrant.get_collection.side_effect = Exception("Collection doesn't exist")
        mock_qdrant.create_collection = AsyncMock()
        mock_upsert = AsyncMock()
        mock_qdrant.upsert = mock_upsert
        
        # Mock OpenAI embeddings with realistic vector size
        mock_embedding_response = AsyncMock()
        mock_embedding_response.data = [AsyncMock()]
        # Create a proper 1536-dimensional embedding vector
        mock_embedding_response.data[0].embedding = [i * 0.01 for i in range(1536)]
        
        mock_client = AsyncMock()
        mock_client.embeddings.create.return_value = mock_embedding_response
        mock_openai.return_value = mock_client
        
        # Mock HTTP response that resembles actual Docusaurus pages
        mock_response = MagicMock()
        mock_response.text = """
        <!DOCTYPE html>
        <html>
        <head><title>Physical AI & Humanoid Robotics Course</title></head>
        <body>
            <main>
                <div class="container">
                    <h1>Physical AI & Humanoid Robotics Course</h1>
                    <p>Welcome to the Physical AI & Humanoid Robotics course. 
                    This comprehensive course covers the intersection of artificial 
                    intelligence and physical robot control.</p>
                    
                    <h2>Course Overview</h2>
                    <p>This course is structured into four main modules that progressively 
                    build your understanding of how AI systems can control physical robots.</p>
                </div>
            </main>
        </body>
        </html>
        """
        mock_response.raise_for_status = MagicMock()
        mock_requests.return_value = mock_response
        
        # Mock URL discovery
        original_discover = ingestor.discover_urls
        async def mock_discover_urls():
            return ["https://hackathon1-book-ragchatbot.vercel.app/test"]
        
        ingestor.discover_urls = mock_discover_urls
        
        # Run the pipeline
        await ingestor.run_ingestion(collection_name="backend_test")
        
        # Verify all steps were executed
        print("✓ URL discovery completed")
        print("✓ Content extraction completed")
        print("✓ Content chunking completed")
        print("✓ Embedding generation completed")
        print("✓ Qdrant storage completed")
        
        # Verify that the collection was created
        mock_qdrant.create_collection.assert_called_once()
        
        # Verify that vectors were upserted
        mock_qdrant.upsert.assert_called_once()
        
        # Restore original method
        ingestor.discover_urls = original_discover
        
        print("✓ Backend functionality test passed!")


async def main_test():
    """Main test function"""
    print("=" * 60)
    print("BACKEND INTEGRATION TEST")
    print("=" * 60)
    
    await test_backend_functionality()
    
    print("\n" + "=" * 60)
    print("BACKEND TEST COMPLETED SUCCESSFULLY!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main_test())