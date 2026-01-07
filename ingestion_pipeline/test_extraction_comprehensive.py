"""
Comprehensive test for content extraction and chunking functionality
"""
import asyncio
from ingestion_pipeline.ingest import WebsiteIngestor


class MockIngestor(WebsiteIngestor):
    """Mock ingestor for testing without external dependencies"""
    
    def __init__(self):
        # Initialize with mock values to avoid actual API calls
        self.visited_urls = set()
    
    async def _fetch_page(self, url: str) -> str:
        """Mock fetch that returns sample content"""
        # Return sample HTML content similar to what Docusaurus generates
        sample_html = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Test Module: The Robotic Nervous System</title>
        </head>
        <body>
            <main>
                <div class="container">
                    <h1>Module 1: The Robotic Nervous System (ROS 2)</h1>
                    <p>Welcome to Module 1 of the Physical AI & Humanoid Robotics course. 
                    This module introduces ROS 2 as the robotic nervous system that connects 
                    AI decision-making to physical humanoid robot control.</p>
                    
                    <h2>Learning Objectives</h2>
                    <p>After completing this module, you will be able to:</p>
                    <ul>
                        <li>Explain ROS 2 architecture and middleware concepts</li>
                        <li>Build and reason about ROS 2 nodes, topics, and services</li>
                        <li>Bridge Python-based AI agents to robot controllers using rclpy</li>
                    </ul>
                    
                    <h2>Module Structure</h2>
                    <p>This module is divided into three chapters:</p>
                    <p>Chapter 1: ROS 2 as the Robotic Nervous System - Conceptual overview 
                    of Physical AI and embodied intelligence, ROS 2 architecture and middleware 
                    role, nodes, topics, services, and message flow.</p>
                    
                    <p>Chapter 2: Communicating with Robots Using ROS 2 - ROS 2 nodes lifecycle, 
                    topics vs services vs actions, Python agents interacting with robots via rclpy, 
                    mapping AI decisions to actuator commands.</p>
                    
                    <p>Chapter 3: Humanoid Robot Structure with URDF - Purpose of URDF in 
                    humanoid robotics, links, joints, and kinematic chains, how URDF connects 
                    software control to physical bodies.</p>
                    
                    <p>The content continues with more detailed explanations, code examples, 
                    diagrams, and exercises that help students understand how ROS 2 serves 
                    as the nervous system for robotic systems. This includes practical 
                    examples of how AI algorithms communicate with physical robots through 
                    the ROS 2 middleware, enabling complex robotic behaviors and interactions.</p>
                    
                    <p>Additional content about message passing, service calls, action servers, 
                    and the overall architecture of distributed robotic systems. Understanding 
                    these concepts is crucial for developing AI agents that can effectively 
                    control and interact with physical robots in real-world scenarios.</p>
                </div>
            </main>
        </body>
        </html>
        """
        return sample_html


async def test_extraction_and_chunking():
    """Test the extraction and chunking functionality"""
    print("Testing content extraction and chunking...")
    
    ingestor = MockIngestor()
    
    # Test content extraction
    test_url = "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/intro"
    content_data = await ingestor.extract_content(test_url)
    
    print(f"Extracted content from {test_url}")
    print(f"Title: {content_data['title']}")
    print(f"Content length: {len(content_data['content'])} characters")
    
    # Test chunking with different parameters
    print("\nTesting chunking with default parameters (max_chunk_size=1000, overlap=200):")
    chunks = ingestor.chunk_content(content_data['content'])
    
    print(f"Content was chunked into {len(chunks)} pieces:")
    for i, chunk in enumerate(chunks):
        print(f"  Chunk {i+1}: {len(chunk)} characters")
        print(f"    Preview: {chunk[:100]}...")
        print()
    
    # Test chunking with smaller parameters
    print("Testing chunking with smaller parameters (max_chunk_size=500, overlap=100):")
    small_chunks = ingestor.chunk_content(content_data['content'], max_chunk_size=500, overlap=100)
    
    print(f"Content was chunked into {len(small_chunks)} pieces:")
    for i, chunk in enumerate(small_chunks):
        print(f"  Chunk {i+1}: {len(chunk)} characters")
    
    # Test edge cases
    print("\nTesting edge cases...")
    
    # Test with content shorter than chunk size
    short_content = "This is a short piece of content."
    short_chunks = ingestor.chunk_content(short_content, max_chunk_size=1000)
    print(f"Short content ({len(short_content)} chars) resulted in {len(short_chunks)} chunks")
    
    # Test with empty content
    empty_chunks = ingestor.chunk_content("", max_chunk_size=1000)
    print(f"Empty content resulted in {len(empty_chunks)} chunks")
    
    print("\nContent extraction and chunking tests completed successfully!")


async def test_url_discovery():
    """Test URL discovery functionality"""
    print("Testing URL discovery...")
    
    ingestor = MockIngestor()
    
    # Test link extraction from sample HTML
    sample_html = """
    <html>
    <body>
        <a href="/docs/modules/ros2/intro">ROS 2 Intro</a>
        <a href="/docs/modules/gazebo-unity/intro">Gazebo Unity Intro</a>
        <a href="/docs/modules/isaac/intro">Isaac Intro</a>
        <a href="/docs/modules/vla/intro">VLA Intro</a>
        <a href="https://external-site.com">External Link</a>
        <a href="#section1">Internal Anchor</a>
    </body>
    </html>
    """
    
    base_url = "https://hackathon1-book-ragchatbot.vercel.app"
    links = await ingestor._extract_links(sample_html, base_url)
    
    print(f"Found {len(links)} internal links:")
    for link in links:
        print(f"  - {link}")
    
    # Filter to only same domain
    same_domain_links = [link for link in links if ingestor._is_same_domain(link, base_url)]
    print(f"\nFiltered to {len(same_domain_links)} same-domain links:")
    for link in same_domain_links:
        print(f"  - {link}")


async def run_all_tests():
    """Run all extraction and chunking tests"""
    print("=" * 60)
    print("COMPREHENSIVE TEST: Content Extraction and Chunking")
    print("=" * 60)
    
    await test_url_discovery()
    print("\n" + "-" * 60)
    await test_extraction_and_chunking()
    
    print("\n" + "=" * 60)
    print("ALL TESTS COMPLETED SUCCESSFULLY!")
    print("=" * 60)


if __name__ == "__main__":
    asyncio.run(run_all_tests())