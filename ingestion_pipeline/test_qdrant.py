"""
Test script for Qdrant Cloud integration and vector storage
"""
import asyncio
from unittest.mock import AsyncMock, patch
from ingestion_pipeline.ingest import WebsiteIngestor


async def test_qdrant_integration():
    """Test Qdrant integration functionality"""
    print("Testing Qdrant integration...")
    
    # Create a mock ingestor
    ingestor = WebsiteIngestor(
        base_url="https://example.com",
        qdrant_url="test-url",
        qdrant_api_key="test-key",
        openai_api_key="test-openai-key"
    )
    
    # Sample chunks to test storage
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
        # First call will raise exception (collection doesn't exist), second will succeed
        mock_qdrant.get_collection.side_effect = [Exception("Collection doesn't exist"), None]
        mock_qdrant.create_collection = AsyncMock()
        mock_qdrant.upsert = AsyncMock()
        
        # Mock the embedding generation
        with patch('openai.AsyncOpenAI') as mock_openai:
            mock_embedding_response = AsyncMock()
            mock_embedding_response.data = [AsyncMock()]
            mock_embedding_response.data[0].embedding = [0.1] * 1536  # 1536 dimensions
            
            mock_client = AsyncMock()
            mock_client.embeddings.create.return_value = mock_embedding_response
            mock_openai.return_value = mock_client
            
            # Call the storage function
            await ingestor.store_in_qdrant(sample_chunks, "test_collection")
            
            # Verify that collection creation was called
            mock_qdrant.create_collection.assert_called_once()
            
            # Verify that upsert was called
            mock_qdrant.upsert.assert_called_once()
            
            # Check that the correct collection name was used
            args, kwargs = mock_qdrant.upsert.call_args
            assert kwargs.get('collection_name') == "test_collection"
            
            print("✓ Qdrant integration test passed!")
            print(f"✓ Collection creation called: {mock_qdrant.create_collection.called}")
            print(f"✓ Vector upsert called: {mock_qdrant.upsert.called}")


async def test_qdrant_existing_collection():
    """Test Qdrant integration when collection already exists"""
    print("Testing Qdrant with existing collection...")
    
    ingestor = WebsiteIngestor(
        base_url="https://example.com",
        qdrant_url="test-url",
        qdrant_api_key="test-key",
        openai_api_key="test-openai-key"
    )
    
    sample_chunks = [
        {
            'url': 'https://example.com/page1',
            'title': 'Test Page 1',
            'content': 'This is test content.',
            'chunk_index': 0
        }
    ]
    
    # Mock Qdrant client where collection already exists
    with patch.object(ingestor, 'qdrant_client') as mock_qdrant:
        # Collection already exists, so get_collection succeeds
        mock_qdrant.get_collection.return_value = {"status": "ok"}
        mock_qdrant.create_collection = AsyncMock()
        mock_qdrant.upsert = AsyncMock()
        
        # Mock embedding generation
        with patch('openai.AsyncOpenAI') as mock_openai:
            mock_embedding_response = AsyncMock()
            mock_embedding_response.data = [AsyncMock()]
            mock_embedding_response.data[0].embedding = [0.1] * 1536
            
            mock_client = AsyncMock()
            mock_client.embeddings.create.return_value = mock_embedding_response
            mock_openai.return_value = mock_client
            
            await ingestor.store_in_qdrant(sample_chunks, "existing_collection")
            
            # Verify that collection creation was NOT called
            assert not mock_qdrant.create_collection.called, "Collection creation should not be called for existing collection"
            
            # Verify that upsert was called
            mock_qdrant.upsert.assert_called_once()
            
            print("✓ Qdrant existing collection test passed!")
            print(f"✓ Collection creation NOT called: {not mock_qdrant.create_collection.called}")
            print(f"✓ Vector upsert called: {mock_qdrant.upsert.called}")


async def test_qdrant_payload_structure():
    """Test that the payload structure is correct for Qdrant storage"""
    print("Testing Qdrant payload structure...")
    
    ingestor = WebsiteIngestor(
        base_url="https://example.com",
        qdrant_url="test-url",
        qdrant_api_key="test-key",
        openai_api_key="test-openai-key"
    )
    
    sample_chunks = [
        {
            'url': 'https://example.com/page1',
            'title': 'Test Page 1',
            'content': 'This is test content.',
            'chunk_index': 0
        }
    ]
    
    # Mock Qdrant client
    with patch.object(ingestor, 'qdrant_client') as mock_qdrant:
        mock_qdrant.get_collection.side_effect = Exception("Collection doesn't exist")
        mock_qdrant.create_collection = AsyncMock()
        mock_qdrant.upsert = AsyncMock()
        
        # Mock embedding generation
        with patch('openai.AsyncOpenAI') as mock_openai:
            mock_embedding_response = AsyncMock()
            mock_embedding_response.data = [AsyncMock()]
            mock_embedding_response.data[0].embedding = [0.1] * 1536
            
            mock_client = AsyncMock()
            mock_client.embeddings.create.return_value = mock_embedding_response
            mock_openai.return_value = mock_client
            
            await ingestor.store_in_qdrant(sample_chunks, "payload_test_collection")
            
            # Get the arguments passed to upsert to check payload structure
            args, kwargs = mock_qdrant.upsert.call_args
            points = kwargs.get('points', args[1] if len(args) > 1 else args[0])
            
            if points:
                point = points[0]  # Get the first point
                payload = point.payload
                
                # Check that required fields are present
                assert 'url' in payload, "Payload should contain 'url'"
                assert 'title' in payload, "Payload should contain 'title'"
                assert 'content' in payload, "Payload should contain 'content'"
                assert 'chunk_index' in payload, "Payload should contain 'chunk_index'"
                
                print(f"✓ Payload contains required fields: {list(payload.keys())}")
                print(f"✓ URL: {payload['url']}")
                print(f"✓ Title: {payload['title']}")
                print(f"✓ Content preview: {payload['content'][:50]}...")
                print(f"✓ Chunk index: {payload['chunk_index']}")
            
            print("✓ Qdrant payload structure test passed!")


async def run_qdrant_tests():
    """Run all Qdrant tests"""
    print("=" * 60)
    print("QDRANT CLOUD INTEGRATION TESTS")
    print("=" * 60)
    
    await test_qdrant_integration()
    print("\n" + "-" * 60)
    await test_qdrant_existing_collection()
    print("\n" + "-" * 60)
    await test_qdrant_payload_structure()
    
    print("\n" + "=" * 60)
    print("ALL QDRANT TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(run_qdrant_tests())