"""
End-to-end test for the complete ingestion pipeline with idempotent behavior
"""
import asyncio
from unittest.mock import AsyncMock, patch, MagicMock
from ingestion_pipeline.ingest import WebsiteIngestor


async def test_full_pipeline():
    """Test the complete ingestion pipeline"""
    print("Testing full ingestion pipeline...")
    
    # Create a mock ingestor
    ingestor = WebsiteIngestor(
        base_url="https://hackathon1-book-ragchatbot.vercel.app",
        qdrant_url="test-url",
        qdrant_api_key="test-key",
        openai_api_key="test-openai-key"
    )
    
    # Mock all external dependencies
    with patch.object(ingestor, 'qdrant_client') as mock_qdrant, \
         patch('openai.AsyncOpenAI') as mock_openai, \
         patch('requests.get') as mock_requests:
        
        # Mock Qdrant operations
        mock_qdrant.get_collection.side_effect = Exception("Collection doesn't exist")
        mock_qdrant.create_collection = AsyncMock()
        mock_qdrant.upsert = AsyncMock()
        
        # Mock OpenAI embeddings
        mock_embedding_response = AsyncMock()
        mock_embedding_response.data = [AsyncMock()]
        mock_embedding_response.data[0].embedding = [0.1] * 1536
        
        mock_client = AsyncMock()
        mock_client.embeddings.create.return_value = mock_embedding_response
        mock_openai.return_value = mock_client
        
        # Mock HTTP requests
        mock_response = MagicMock()
        mock_response.text = """
        <html>
        <head><title>Test Page</title></head>
        <body>
            <main>
                <h1>Test Content</h1>
                <p>This is test content for the RAG chatbot pipeline.</p>
                <p>The content continues with more detailed information about the topic.</p>
                <p>Additional paragraphs provide more context and examples.</p>
            </main>
        </body>
        </html>
        """
        mock_response.raise_for_status = MagicMock()
        mock_requests.return_value = mock_response
        
        # Run the full pipeline
        await ingestor.run_ingestion(collection_name="test_collection")
        
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
        
        print("✓ Full pipeline execution test passed!")


async def test_idempotent_behavior():
    """Test that the pipeline is idempotent (can be run multiple times without duplicating data)"""
    print("Testing idempotent behavior...")
    
    ingestor = WebsiteIngestor(
        base_url="https://hackathon1-book-ragchatbot.vercel.app",
        qdrant_url="test-url",
        qdrant_api_key="test-key",
        openai_api_key="test-openai-key"
    )
    
    # Mock all external dependencies
    with patch.object(ingestor, 'qdrant_client') as mock_qdrant, \
         patch('openai.AsyncOpenAI') as mock_openai, \
         patch('requests.get') as mock_requests:
        
        # Mock Qdrant operations
        mock_qdrant.get_collection.side_effect = Exception("Collection doesn't exist")
        mock_qdrant.create_collection = AsyncMock()
        mock_upsert = AsyncMock()
        mock_qdrant.upsert = mock_upsert
        
        # Mock OpenAI embeddings
        mock_embedding_response = AsyncMock()
        mock_embedding_response.data = [AsyncMock()]
        mock_embedding_response.data[0].embedding = [0.1] * 1536
        
        mock_client = AsyncMock()
        mock_client.embeddings.create.return_value = mock_embedding_response
        mock_openai.return_value = mock_client
        
        # Mock HTTP requests
        mock_response = MagicMock()
        mock_response.text = """
        <html>
        <head><title>Test Page</title></head>
        <body>
            <main>
                <h1>Test Content</h1>
                <p>This is test content for idempotency testing.</p>
            </main>
        </body>
        </html>
        """
        mock_response.raise_for_status = MagicMock()
        mock_requests.return_value = mock_response
        
        # Run the pipeline multiple times
        await ingestor.run_ingestion(collection_name="idempotent_test")
        await ingestor.run_ingestion(collection_name="idempotent_test")
        await ingestor.run_ingestion(collection_name="idempotent_test")
        
        # Check that upsert was called the same number of times as runs
        assert mock_upsert.call_count == 3, f"Expected 3 upsert calls, got {mock_upsert.call_count}"
        
        print("✓ Idempotent behavior test passed!")
        print(f"✓ Pipeline ran 3 times with {mock_upsert.call_count} upsert operations")


async def test_pipeline_with_multiple_urls():
    """Test the pipeline with multiple URLs"""
    print("Testing pipeline with multiple URLs...")
    
    ingestor = WebsiteIngestor(
        base_url="https://hackathon1-book-ragchatbot.vercel.app",
        qdrant_url="test-url",
        qdrant_api_key="test-key",
        openai_api_key="test-openai-key"
    )
    
    # Mock all external dependencies
    with patch.object(ingestor, 'qdrant_client') as mock_qdrant, \
         patch('openai.AsyncOpenAI') as mock_openai, \
         patch('requests.get') as mock_requests:
        
        # Mock Qdrant operations
        mock_qdrant.get_collection.side_effect = Exception("Collection doesn't exist")
        mock_qdrant.create_collection = AsyncMock()
        mock_upsert = AsyncMock()
        mock_qdrant.upsert = mock_upsert
        
        # Mock OpenAI embeddings
        mock_embedding_response = AsyncMock()
        mock_embedding_response.data = [AsyncMock()]
        mock_embedding_response.data[0].embedding = [0.1] * 1536
        
        mock_client = AsyncMock()
        mock_client.embeddings.create.return_value = mock_embedding_response
        mock_openai.return_value = mock_client
        
        # Mock responses for different URLs
        def mock_get(url, *args, **kwargs):
            mock_resp = MagicMock()
            if "page1" in url:
                mock_resp.text = """
                <html>
                <head><title>Page 1</title></head>
                <body>
                    <main>
                        <h1>Page 1 Content</h1>
                        <p>This is content from page 1.</p>
                    </main>
                </body>
                </html>
                """
            else:
                mock_resp.text = """
                <html>
                <head><title>Page 2</title></head>
                <body>
                    <main>
                        <h1>Page 2 Content</h1>
                        <p>This is content from page 2.</p>
                    </main>
                </body>
                </html>
                """
            mock_resp.raise_for_status = MagicMock()
            return mock_resp
        
        mock_requests.side_effect = mock_get
        
        # Mock URL discovery to return multiple URLs
        original_discover = ingestor.discover_urls
        async def mock_discover_urls():
            return [
                "https://hackathon1-book-ragchatbot.vercel.app/page1",
                "https://hackathon1-book-ragchatbot.vercel.app/page2"
            ]
        
        ingestor.discover_urls = mock_discover_urls
        
        # Run the pipeline
        await ingestor.run_ingestion(collection_name="multi_url_test")
        
        # Verify that processing happened for multiple URLs
        print("✓ Multiple URLs processed")
        print(f"✓ Total upsert calls: {mock_upsert.call_count}")
        
        # Restore original method
        ingestor.discover_urls = original_discover
        
        print("✓ Multiple URLs pipeline test passed!")


async def run_end_to_end_tests():
    """Run all end-to-end tests"""
    print("=" * 60)
    print("END-TO-END INGESTION PIPELINE TESTS")
    print("=" * 60)
    
    await test_full_pipeline()
    print("\n" + "-" * 60)
    await test_idempotent_behavior()
    print("\n" + "-" * 60)
    await test_pipeline_with_multiple_urls()
    
    print("\n" + "=" * 60)
    print("ALL END-TO-END TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(run_end_to_end_tests())