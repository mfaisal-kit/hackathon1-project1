"""
Mock Retrieval Pipeline and Validation for RAG Chatbot

This module implements a mock version of the retrieval pipeline that simulates
the behavior without requiring OpenAI API calls, for testing when API quotas are exceeded.
"""
import os
import asyncio
import logging
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
import numpy as np

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MockRetrievalPipeline:
    """
    Class to handle the retrieval pipeline with mock embeddings for testing
    """
    
    def __init__(self):
        """
        Initialize the mock retrieval pipeline with Qdrant client
        """
        self.qdrant_client = AsyncQdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.collection_name = "book_content"
        
    async def validate_connection(self) -> bool:
        """
        Validate connection to Qdrant and check if collection exists
        """
        try:
            collection_info = await self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Connected to Qdrant collection: {self.collection_name}")
            logger.info(f"Collection points count: {collection_info.points_count}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant collection: {str(e)}")
            return False
    
    def generate_mock_embedding(self, text: str) -> List[float]:
        """
        Generate a mock embedding for the input text
        This creates a deterministic vector based on the text content
        """
        # Create a hash-based embedding to ensure consistency
        text_hash = hash(text) % (2**32)
        np.random.seed(text_hash)
        
        # Generate a 1536-dimensional vector (same as OpenAI's text-embedding-3-small)
        embedding = np.random.random(1536).tolist()
        
        # Normalize the vector to unit length
        norm = np.linalg.norm(embedding)
        if norm > 0:
            embedding = (np.array(embedding) / norm).tolist()
        
        return embedding
    
    async def search_similar_chunks(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in Qdrant based on the query embedding
        """
        try:
            search_results = await self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True,
                with_vectors=False
            )
            
            # Format results
            formatted_results = []
            for result in search_results:
                formatted_result = {
                    'id': result.id,
                    'content': result.payload.get('content', ''),
                    'url': result.payload.get('url', ''),
                    'title': result.payload.get('title', ''),
                    'chunk_index': result.payload.get('chunk_index', 0),
                    'score': result.score
                }
                formatted_results.append(formatted_result)
            
            logger.info(f"Retrieved {len(formatted_results)} similar chunks for query")
            return formatted_results
        except Exception as e:
            logger.error(f"Failed to search similar chunks: {str(e)}")
            return []
    
    async def validate_retrieved_chunks(self, chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate the retrieved chunks for accuracy and consistency
        """
        validation_report = {
            'total_chunks': len(chunks),
            'valid_chunks': 0,
            'invalid_chunks': 0,
            'missing_metadata': 0,
            'valid_urls': 0,
            'invalid_urls': 0,
            'validation_details': []
        }
        
        for chunk in chunks:
            is_valid = True
            chunk_validation = {
                'id': chunk.get('id'),
                'valid_content': bool(chunk.get('content')),
                'valid_url': bool(chunk.get('url')),
                'valid_title': bool(chunk.get('title')),
                'has_chunk_index': 'chunk_index' in chunk,
                'issues': []
            }
            
            # Check for content
            if not chunk.get('content'):
                is_valid = False
                chunk_validation['issues'].append('Missing content')
            
            # Check for URL
            if not chunk.get('url'):
                is_valid = False
                chunk_validation['issues'].append('Missing URL')
            else:
                # Validate URL format
                if not chunk['url'].startswith(('http://', 'https://')):
                    chunk_validation['issues'].append('Invalid URL format')
                    is_valid = False
            
            # Check for title
            if not chunk.get('title'):
                chunk_validation['issues'].append('Missing title')
            
            # Update validation report
            if is_valid:
                validation_report['valid_chunks'] += 1
                if chunk.get('url'):
                    validation_report['valid_urls'] += 1
            else:
                validation_report['invalid_chunks'] += 1
                if not chunk.get('url'):
                    validation_report['invalid_urls'] += 1
            
            if not chunk.get('title') or 'chunk_index' not in chunk:
                validation_report['missing_metadata'] += 1
            
            validation_report['validation_details'].append(chunk_validation)
        
        return validation_report
    
    async def retrieve_and_validate(self, query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Complete retrieval and validation pipeline using mock embeddings
        """
        logger.info(f"Starting retrieval for query: {query}")
        
        # Step 1: Generate mock query embedding
        query_embedding = self.generate_mock_embedding(query)
        
        # Step 2: Search for similar chunks
        retrieved_chunks = await self.search_similar_chunks(query_embedding, top_k)
        
        # Step 3: Validate retrieved chunks
        validation_report = await self.validate_retrieved_chunks(retrieved_chunks)
        
        return {
            'query': query,
            'retrieved_chunks': retrieved_chunks,
            'validation_report': validation_report,
            'success': True
        }


class MockRetrievalTester:
    """
    Class to test the mock retrieval pipeline with predefined queries
    """
    
    def __init__(self, retrieval_pipeline: MockRetrievalPipeline):
        self.pipeline = retrieval_pipeline
        self.test_queries = [
            "What is ROS 2?",
            "Explain humanoid robotics",
            "How does Gazebo simulation work?",
            "What is NVIDIA Isaac?",
            "Vision-Language-Action systems"
        ]
    
    async def run_retrieval_test(self) -> Dict[str, Any]:
        """
        Run comprehensive test of the mock retrieval pipeline
        """
        logger.info("Starting mock retrieval pipeline test...")
        
        test_results = {
            'total_queries': len(self.test_queries),
            'successful_retrievals': 0,
            'failed_retrievals': 0,
            'average_chunks_per_query': 0,
            'total_chunks_retrieved': 0,
            'validation_summary': {
                'total_chunks': 0,
                'valid_chunks': 0,
                'invalid_chunks': 0,
                'valid_urls': 0,
                'invalid_urls': 0
            },
            'query_results': []
        }
        
        all_chunks = []
        
        for i, query in enumerate(self.test_queries):
            logger.info(f"Testing query {i+1}/{len(self.test_queries)}: {query}")
            
            result = await self.pipeline.retrieve_and_validate(query)
            
            if result['success']:
                test_results['successful_retrievals'] += 1
                test_results['total_chunks_retrieved'] += len(result['retrieved_chunks'])
                all_chunks.extend(result['retrieved_chunks'])
                
                # Update validation summary
                if result['validation_report']:
                    val_report = result['validation_report']
                    test_results['validation_summary']['total_chunks'] += val_report['total_chunks']
                    test_results['validation_summary']['valid_chunks'] += val_report['valid_chunks']
                    test_results['validation_summary']['invalid_chunks'] += val_report['invalid_chunks']
                    test_results['validation_summary']['valid_urls'] += val_report['valid_urls']
                    test_results['validation_summary']['invalid_urls'] += val_report['invalid_urls']
            else:
                test_results['failed_retrievals'] += 1
            
            test_results['query_results'].append(result)
        
        # Calculate averages
        if test_results['successful_retrievals'] > 0:
            test_results['average_chunks_per_query'] = (
                test_results['total_chunks_retrieved'] / test_results['successful_retrievals']
            )
        
        return test_results
    
    async def generate_test_report(self) -> str:
        """
        Generate a comprehensive test report
        """
        test_results = await self.run_retrieval_test()
        
        report = []
        report.append("=" * 60)
        report.append("MOCK RETRIEVAL PIPELINE TEST REPORT")
        report.append("=" * 60)
        report.append(f"Total Queries: {test_results['total_queries']}")
        report.append(f"Successful Retrievals: {test_results['successful_retrievals']}")
        report.append(f"Failed Retrievals: {test_results['failed_retrievals']}")
        report.append(f"Total Chunks Retrieved: {test_results['total_chunks_retrieved']}")
        report.append(f"Average Chunks per Query: {test_results['average_chunks_per_query']:.2f}")
        report.append("")
        
        # Validation Summary
        val_summary = test_results['validation_summary']
        report.append("VALIDATION SUMMARY:")
        report.append(f"  Total Chunks: {val_summary['total_chunks']}")
        report.append(f"  Valid Chunks: {val_summary['valid_chunks']}")
        report.append(f"  Invalid Chunks: {val_summary['invalid_chunks']}")
        report.append(f"  Valid URLs: {val_summary['valid_urls']}")
        report.append(f"  Invalid URLs: {val_summary['invalid_urls']}")
        report.append("")
        
        # Individual Query Results
        report.append("INDIVIDUAL QUERY RESULTS:")
        for i, result in enumerate(test_results['query_results']):
            report.append(f"  Query {i+1}: {result['query']}")
            report.append(f"    Success: {result['success']}")
            report.append(f"    Chunks Retrieved: {len(result['retrieved_chunks'])}")
            
            if result['validation_report']:
                val_report = result['validation_report']
                report.append(f"    Valid Chunks: {val_report['valid_chunks']}")
                report.append(f"    Invalid Chunks: {val_report['invalid_chunks']}")
            
            report.append("")
        
        report.append("=" * 60)
        report.append("TEST COMPLETED")
        report.append("=" * 60)
        
        return "\n".join(report)


async def main():
    """
    Main function to demonstrate the mock retrieval pipeline
    """
    logger.info("Initializing Mock Retrieval Pipeline...")
    
    # Initialize the mock retrieval pipeline
    retrieval_pipeline = MockRetrievalPipeline()
    
    # Validate connection
    is_connected = await retrieval_pipeline.validate_connection()
    if not is_connected:
        logger.error("Failed to connect to Qdrant. Please check your configuration.")
        return
    
    # Create tester
    tester = MockRetrievalTester(retrieval_pipeline)
    
    # Run comprehensive test
    test_report = await tester.generate_test_report()
    print(test_report)
    
    # Example of single query retrieval
    logger.info("\nTesting single query retrieval...")
    sample_query = "What is the robotic nervous system?"
    result = await retrieval_pipeline.retrieve_and_validate(sample_query, top_k=3)
    
    print(f"\nQuery: {sample_query}")
    print(f"Success: {result['success']}")
    print(f"Retrieved {len(result['retrieved_chunks'])} chunks:")
    
    for i, chunk in enumerate(result['retrieved_chunks']):
        print(f"  Chunk {i+1}:")
        print(f"    URL: {chunk['url']}")
        print(f"    Title: {chunk['title']}")
        print(f"    Score: {chunk['score']:.4f}")
        print(f"    Content Preview: {chunk['content'][:100]}...")
        print()


if __name__ == "__main__":
    asyncio.run(main())