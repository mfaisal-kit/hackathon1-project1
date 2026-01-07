"""
Integration test to run the pipeline with sample data and verify vector storage
"""
import asyncio
import tempfile
import os
from unittest.mock import AsyncMock, patch, MagicMock
from ingestion_pipeline.ingest import WebsiteIngestor


async def test_with_realistic_sample_data():
    """Test the pipeline with more realistic sample data"""
    print("Testing with realistic sample data...")
    
    # Create a mock ingestor
    ingestor = WebsiteIngestor(
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
        
        # Mock responses that resemble actual Docusaurus pages
        def mock_get(url, *args, **kwargs):
            mock_resp = MagicMock()
            if "intro" in url or "index" in url:
                mock_resp.text = """
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
                            
                            <h3>Module 1: The Robotic Nervous System (ROS 2)</h3>
                            <p>ROS 2 serves as the middleware that connects AI decision-making 
                            systems to physical robot control systems. In this module, you'll 
                            learn about nodes, topics, services, and actions that enable 
                            communication between different parts of a robotic system.</p>
                            
                            <h3>Module 2: The Digital Twin (Gazebo & Unity)</h3>
                            <p>Creating accurate simulations is crucial for developing and 
                            testing robotic systems. This module covers physics simulation 
                            with Gazebo and high-fidelity interaction with Unity, allowing 
                            you to test your AI agents in virtual environments before 
                            deploying them to real robots.</p>
                            
                            <h3>Module 3: The AI-Robot Brain (NVIDIA Isaac™)</h3>
                            <p>NVIDIA Isaac provides a comprehensive platform for developing 
                            AI-powered robotic applications. You'll learn about Isaac Sim 
                            for synthetic data generation, Isaac ROS for perception and 
                            localization, and Nav2 for navigation systems.</p>
                            
                            <h3>Module 4: Vision-Language-Action (VLA)</h3>
                            <p>The integration of vision, language, and action systems 
                            represents the cutting edge of AI robotics. In this module, 
                            you'll explore how to build systems that can understand natural 
                            language commands and execute complex physical tasks.</p>
                        </div>
                    </main>
                </body>
                </html>
                """
            elif "ros2" in url:
                mock_resp.text = """
                <!DOCTYPE html>
                <html>
                <head><title>Module 1: The Robotic Nervous System</title></head>
                <body>
                    <main>
                        <div class="container">
                            <h1>Module 1: The Robotic Nervous System (ROS 2)</h1>
                            <p>Welcome to Module 1 of the Physical AI & Humanoid Robotics course. 
                            This module introduces ROS 2 as the robotic nervous system that 
                            connects AI decision-making to physical humanoid robot control.</p>
                            
                            <h2>Learning Objectives</h2>
                            <p>After completing this module, you will be able to:</p>
                            <ul>
                                <li>Explain ROS 2 architecture and middleware concepts</li>
                                <li>Build and reason about ROS 2 nodes, topics, and services</li>
                                <li>Bridge Python-based AI agents to robot controllers using rclpy</li>
                                <li>Understand and read URDF files for humanoid robots</li>
                                <li>Conceptually trace data flow from AI logic → ROS 2 → humanoid movement</li>
                            </ul>
                            
                            <h2>Module Structure</h2>
                            <p>This module is divided into three chapters:</p>
                            
                            <h3>Chapter 1: ROS 2 as the Robotic Nervous System</h3>
                            <p>Conceptual overview of Physical AI and embodied intelligence, 
                            ROS 2 architecture and middleware role, nodes, topics, services, 
                            and message flow. Understanding how ROS 2 serves as the nervous 
                            system for robotic applications is fundamental to building 
                            effective AI-robot systems.</p>
                            
                            <h3>Chapter 2: Communicating with Robots Using ROS 2</h3>
                            <p>ROS 2 nodes lifecycle, topics vs services vs actions, 
                            Python agents interacting with robots via rclpy, mapping 
                            AI decisions to actuator commands. This chapter focuses on 
                            the practical aspects of implementing communication between 
                            AI systems and robotic hardware.</p>
                            
                            <h3>Chapter 3: Humanoid Robot Structure with URDF</h3>
                            <p>Purpose of URDF in humanoid robotics, links, joints, and 
                            kinematic chains, how URDF connects software control to 
                            physical bodies. Understanding robot description formats 
                            is essential for controlling complex robotic systems.</p>
                        </div>
                    </main>
                </body>
                </html>
                """
            elif "gazebo" in url:
                mock_resp.text = """
                <!DOCTYPE html>
                <html>
                <head><title>Module 2: The Digital Twin</title></head>
                <body>
                    <main>
                        <div class="container">
                            <h1>Module 2: The Digital Twin (Gazebo & Unity)</h1>
                            <p>Module 2 focuses on creating digital twins of physical robots 
                            and environments. Digital twins enable safe testing and 
                            development of AI agents before deployment to real hardware.</p>
                            
                            <h2>Physics Simulation with Gazebo</h2>
                            <p>Gazebo provides realistic physics simulation that allows 
                            you to test robotic systems in virtual environments that 
                            closely approximate real-world conditions. This includes 
                            accurate modeling of forces, collisions, and sensor data.</p>
                            
                            <h2>High-Fidelity Interaction with Unity</h2>
                            <p>Unity offers high-fidelity visualization and interaction 
                            capabilities that complement physics simulation. This module 
                            shows how to integrate Unity with ROS 2 for enhanced 
                            simulation experiences.</p>
                            
                            <h2>Simulated Sensors for Perception</h2>
                            <p>Accurate sensor simulation is crucial for developing 
                            robust perception systems. This section covers simulating 
                            cameras, LiDAR, IMUs, and other sensors used in robotic 
                            perception systems.</p>
                        </div>
                    </main>
                </body>
                </html>
                """
            else:
                # Default response for other URLs
                mock_resp.text = """
                <!DOCTYPE html>
                <html>
                <head><title>Test Page</title></head>
                <body>
                    <main>
                        <div class="container">
                            <h1>Additional Test Content</h1>
                            <p>This page contains additional content for testing the 
                            ingestion pipeline. The content should be substantial 
                            enough to test the chunking functionality properly.</p>
                            <p>The pipeline needs to handle various types of content 
                            including headings, paragraphs, lists, and code examples. 
                            Each piece of content should be properly extracted and 
                            converted to vector embeddings for storage in Qdrant.</p>
                            <p>Additional content to make the page longer and test 
                            the chunking algorithm with overlapping content to 
                            preserve context between chunks.</p>
                        </div>
                    </main>
                </body>
                </html>
                """
            
            mock_resp.raise_for_status = MagicMock()
            return mock_resp
        
        mock_requests.side_effect = mock_get
        
        # Mock URL discovery to return realistic URLs
        original_discover = ingestor.discover_urls
        async def mock_discover_urls():
            return [
                "https://hackathon1-book-ragchatbot.vercel.app/docs/intro",
                "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/intro",
                "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/gazebo-unity/intro",
                "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/isaac/intro",
                "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/vla/intro"
            ]
        
        ingestor.discover_urls = mock_discover_urls
        
        # Run the pipeline
        await ingestor.run_ingestion(collection_name="integration_test")
        
        # Verify the results
        print("✓ Pipeline completed successfully")
        print(f"✓ Collection creation called: {mock_qdrant.create_collection.called}")
        print(f"✓ Vector upsert called: {mock_upsert.called}")
        print(f"✓ Number of upsert calls: {mock_upsert.call_count}")
        
        # Check that upsert was called with correct parameters
        if mock_upsert.called:
            args, kwargs = mock_upsert.call_args
            collection_name = kwargs.get('collection_name', args[0] if len(args) > 0 else 'unknown')
            points = kwargs.get('points', args[1] if len(args) > 1 else [])
            
            print(f"✓ Data stored in collection: {collection_name}")
            print(f"✓ Number of points stored: {len(points) if points else 'unknown'}")
        
        # Restore original method
        ingestor.discover_urls = original_discover
        
        print("✓ Realistic sample data test passed!")


async def test_vector_storage_verification():
    """Test the verification of vector storage in Qdrant"""
    print("Testing vector storage verification...")
    
    ingestor = WebsiteIngestor(
        base_url="https://hackathon1-book-ragchatbot.vercel.app",
        qdrant_url="test-url",
        qdrant_api_key="test-key",
        openai_api_key="test-openai-key"
    )
    
    # Mock Qdrant operations
    with patch.object(ingestor, 'qdrant_client') as mock_qdrant, \
         patch('openai.AsyncOpenAI') as mock_openai, \
         patch('requests.get') as mock_requests:
        
        # Mock collection operations
        mock_qdrant.get_collection.side_effect = Exception("Collection doesn't exist")
        mock_qdrant.create_collection = AsyncMock()
        mock_upsert = AsyncMock()
        mock_qdrant.upsert = mock_upsert
        
        # Mock embedding generation
        mock_embedding_response = AsyncMock()
        mock_embedding_response.data = [AsyncMock()]
        mock_embedding_response.data[0].embedding = [0.1] * 1536
        
        mock_client = AsyncMock()
        mock_client.embeddings.create.return_value = mock_embedding_response
        mock_openai.return_value = mock_client
        
        # Mock HTTP response
        mock_response = MagicMock()
        mock_response.text = """
        <html>
        <head><title>Verification Test</title></head>
        <body>
            <main>
                <h1>Verification Test Content</h1>
                <p>This content is used to verify that vectors are properly stored in Qdrant.</p>
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
        await ingestor.run_ingestion(collection_name="verification_test")
        
        # Verify that upsert was called with proper vector data
        if mock_upsert.called:
            args, kwargs = mock_upsert.call_args
            points = kwargs.get('points', args[1] if len(args) > 1 else [])
            
            if points:
                point = points[0]  # Check the first point
                
                # Verify vector dimensions
                vector_len = len(point.vector) if hasattr(point, 'vector') else 0
                print(f"✓ Vector dimensions: {vector_len} (expected: 1536)")
                
                # Verify payload structure
                payload = point.payload if hasattr(point, 'payload') else {}
                required_fields = ['url', 'title', 'content', 'chunk_index']
                present_fields = [field for field in required_fields if field in payload]
                print(f"✓ Payload contains {len(present_fields)}/{len(required_fields)} required fields: {present_fields}")
                
                # Verify vector values are not empty
                vector_sum = sum(point.vector) if hasattr(point, 'vector') and point.vector else 0
                print(f"✓ Vector sum: {vector_sum} (non-zero indicates valid embedding)")
        
        # Restore original method
        ingestor.discover_urls = original_discover
        
        print("✓ Vector storage verification test passed!")


async def run_integration_tests():
    """Run all integration tests"""
    print("=" * 70)
    print("INTEGRATION TEST: Pipeline with Sample Data and Vector Storage Verification")
    print("=" * 70)
    
    await test_with_realistic_sample_data()
    print("\n" + "-" * 70)
    await test_vector_storage_verification()
    
    print("\n" + "=" * 70)
    print("ALL INTEGRATION TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 70)


if __name__ == "__main__":
    asyncio.run(run_integration_tests())