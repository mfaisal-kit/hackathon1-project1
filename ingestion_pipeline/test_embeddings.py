"""
Test script for embedding generation functionality
"""
import asyncio
import os
from unittest.mock import AsyncMock, patch
from ingestion_pipeline.ingest import WebsiteIngestor


async def test_embedding_generation():
    """Test embedding generation functionality"""
    print("Testing embedding generation...")
    
    # Mock API key for testing
    ingestor = WebsiteIngestor(
        base_url="https://example.com",
        qdrant_url="test",
        qdrant_api_key="test",
        openai_api_key="test"
    )
    
    # Test text for embedding
    test_text = "This is a test sentence for embedding generation."
    
    # Since we don't have a real API key in testing, let's mock the OpenAI API call
    with patch('openai.AsyncOpenAI') as mock_openai:
        # Set up the mock to return a fixed embedding
        mock_embedding_response = AsyncMock()
        mock_embedding_response.data = [AsyncMock()]
        mock_embedding_response.data[0].embedding = [0.1, 0.2, 0.3, 0.4, 0.5] * 307  # 1535 dimensions + 1 = 1536
        
        mock_client = AsyncMock()
        mock_client.embeddings.create.return_value = mock_embedding_response
        mock_openai.return_value = mock_client
        
        # Call the embedding function
        embedding = await ingestor.generate_embedding(test_text)
        
        print(f"Generated embedding with {len(embedding)} dimensions")
        print(f"First 5 values: {embedding[:5]}")
        print(f"Last 5 values: {embedding[-5:]}")
        
        # Verify the embedding has the expected dimensions (for text-embedding-3-small)
        assert len(embedding) == 1536, f"Expected 1536 dimensions, got {len(embedding)}"
        
        print("✓ Embedding generation test passed!")
    
    # Test with empty text
    empty_embedding = await ingestor.generate_embedding("")
    print(f"Empty text embedding length: {len(empty_embedding)}")
    
    # Test with longer text
    long_text = "This is a longer text sample. " * 50  # Repeat to make it longer
    with patch('openai.AsyncOpenAI') as mock_openai:
        mock_embedding_response = AsyncMock()
        mock_embedding_response.data = [AsyncMock()]
        mock_embedding_response.data[0].embedding = [0.1] * 1536
        
        mock_client = AsyncMock()
        mock_client.embeddings.create.return_value = mock_embedding_response
        mock_openai.return_value = mock_client
        
        long_embedding = await ingestor.generate_embedding(long_text)
        print(f"Long text embedding length: {len(long_embedding)}")
        assert len(long_embedding) == 1536, f"Expected 1536 dimensions for long text, got {len(long_embedding)}"
        
        print("✓ Long text embedding test passed!")
    
    print("\nEmbedding generation tests completed successfully!")


async def test_embedding_integration():
    """Test embedding integration with the rest of the pipeline"""
    print("Testing embedding integration...")
    
    # Create a mock ingestor
    ingestor = WebsiteIngestor(
        base_url="https://example.com",
        qdrant_url="test",
        qdrant_api_key="test",
        openai_api_key="test"
    )
    
    # Sample content to test the full flow
    sample_chunks = [
        {
            'url': 'https://example.com/page1',
            'title': 'Test Page 1',
            'content': 'This is the first test chunk of content.',
            'chunk_index': 0
        },
        {
            'url': 'https://example.com/page1',
            'title': 'Test Page 1',
            'content': 'This is the second test chunk of content.',
            'chunk_index': 1
        }
    ]
    
    # Mock the Qdrant client
    with patch.object(ingestor, 'qdrant_client') as mock_qdrant:
        # Mock Qdrant collection operations
        mock_qdrant.get_collection.side_effect = Exception("Collection doesn't exist")
        mock_qdrant.create_collection = AsyncMock()
        mock_qdrant.upsert = AsyncMock()
        
        # Mock the embedding generation
        with patch('openai.AsyncOpenAI') as mock_openai:
            mock_embedding_response = AsyncMock()
            mock_embedding_response.data = [AsyncMock()]
            mock_embedding_response.data[0].embedding = [0.1] * 1536
            
            mock_client = AsyncMock()
            mock_client.embeddings.create.return_value = mock_embedding_response
            mock_openai.return_value = mock_client
            
            # Call the storage function which will trigger embedding generation
            await ingestor.store_in_qdrant(sample_chunks, "test_collection")
            
            # Verify that upsert was called (meaning embeddings were generated successfully)
            mock_qdrant.upsert.assert_called_once()
            print("✓ Embedding integration test passed!")


async def run_embedding_tests():
    """Run all embedding tests"""
    print("=" * 60)
    print("EMBEDDING GENERATION TESTS")
    print("=" * 60)
    
    await test_embedding_generation()
    print("\n" + "-" * 60)
    await test_embedding_integration()
    
    print("\n" + "=" * 60)
    print("ALL EMBEDDING TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(run_embedding_tests())