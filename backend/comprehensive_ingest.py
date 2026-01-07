"""
Comprehensive ingestion script to populate Qdrant database with ALL book content
"""
import os
import asyncio
import hashlib
from typing import List, Dict, Any
from urllib.parse import urljoin, urlparse
import requests
from bs4 import BeautifulSoup
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ComprehensiveBookIngestor:
    def __init__(self):
        self.qdrant_client = AsyncQdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.visited_urls = set()
        self.collection_name = "book_content"
    
    async def create_collection_if_not_exists(self):
        """Create collection if it doesn't exist"""
        try:
            await self.qdrant_client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Collection doesn't exist, create it
            await self.qdrant_client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # Standard for text-embedding-3-small
            )
            logger.info(f"Created new Qdrant collection: {self.collection_name}")
    
    async def discover_all_urls(self, base_url: str) -> List[str]:
        """Discover ALL URLs from the Docusaurus website using multiple methods"""
        logger.info(f"Discovering ALL URLs from {base_url}")
        all_urls = set()
        
        try:
            # Method 1: Get URLs from sitemap
            sitemap_url = urljoin(base_url, "sitemap.xml")
            sitemap_urls = await self._extract_from_sitemap(sitemap_url)
            all_urls.update(sitemap_urls)
            logger.info(f"Found {len(sitemap_urls)} URLs from sitemap")
            
            # Method 2: Get URLs from main navigation
            main_page_urls = await self._extract_from_main_page(base_url)
            all_urls.update(main_page_urls)
            logger.info(f"Found {len(main_page_urls)} URLs from main page")
            
            # Method 3: Get URLs from sidebar structure (common in Docusaurus)
            sidebar_urls = await self._extract_from_sidebar(base_url)
            all_urls.update(sidebar_urls)
            logger.info(f"Found {len(sidebar_urls)} URLs from sidebar structure")
            
            # Method 4: Brute force common Docusaurus patterns
            pattern_urls = await self._extract_from_patterns(base_url)
            all_urls.update(pattern_urls)
            logger.info(f"Found {len(pattern_urls)} URLs from patterns")
            
            # Filter URLs
            filtered_urls = [url for url in all_urls 
                           if self._is_same_domain(url, base_url) and 
                           not url.endswith(('.jpg', '.jpeg', '.png', '.gif', '.pdf', '.zip', '.css', '.js', '.ico', '.svg')) and
                           ('/docs/' in url or '/modules/' in url or url == base_url or '/blog/' in url)]
            
            logger.info(f"Total unique URLs found: {len(filtered_urls)}")
            return list(filtered_urls)
            
        except Exception as e:
            logger.error(f"Error discovering URLs: {str(e)}")
            # Return some default URLs if discovery fails
            return [
                base_url,
                f"{base_url}/docs/intro",
                f"{base_url}/docs/modules/ros2/intro",
                f"{base_url}/docs/modules/ros2/chapter-1/introduction-to-ros2-nervous-system",
                f"{base_url}/docs/modules/ros2/chapter-2/communicating-with-robots-using-ros2",
                f"{base_url}/docs/modules/ros2/chapter-3/humanoid-robot-structure-with-urdf",
                f"{base_url}/docs/modules/gazebo-unity/intro",
                f"{base_url}/docs/modules/gazebo-unity/chapter-1/physics-simulation-with-gazebo",
                f"{base_url}/docs/modules/gazebo-unity/chapter-2/high-fidelity-interaction-with-unity",
                f"{base_url}/docs/modules/gazebo-unity/chapter-3/simulated-sensors-for-perception",
                f"{base_url}/docs/modules/isaac/intro",
                f"{base_url}/docs/modules/isaac/chapter-1/nvidia-isaac-sim-synthetic-data",
                f"{base_url}/docs/modules/isaac/chapter-2/isaac-ros-perception-localization",
                f"{base_url}/docs/modules/isaac/chapter-3/nav2-humanoid-navigation",
                f"{base_url}/docs/modules/vla/intro",
                f"{base_url}/docs/modules/vla/chapter-1/voice-to-action-with-speech-models",
                f"{base_url}/docs/modules/vla/chapter-2/language-driven-cognitive-planning",
                f"{base_url}/docs/modules/vla/chapter-3/capstone-autonomous-humanoid"
            ]
    
    async def _extract_from_sitemap(self, sitemap_url: str) -> List[str]:
        """Extract URLs from sitemap if available"""
        try:
            response = requests.get(sitemap_url, timeout=30)
            if response.status_code == 200:
                soup = BeautifulSoup(response.content, 'xml')
                urls = []
                for loc in soup.find_all('loc'):
                    url = loc.text.strip()
                    urls.append(url)
                logger.info(f"Successfully parsed sitemap with {len(urls)} URLs")
                return urls
        except Exception as e:
            logger.warning(f"Could not fetch sitemap: {str(e)}")
        
        return []
    
    async def _extract_from_main_page(self, base_url: str) -> List[str]:
        """Extract URLs from the main page"""
        try:
            response = requests.get(base_url, timeout=30)
            response.raise_for_status()
            
            soup = BeautifulSoup(response.text, 'html.parser')
            urls = []
            
            # Find all links on the page
            for link in soup.find_all('a', href=True):
                href = link['href']
                full_url = urljoin(base_url, href)
                
                # Only include internal links
                if self._is_same_domain(full_url, base_url):
                    urls.append(full_url)
            
            return urls
        except Exception as e:
            logger.warning(f"Could not extract from main page: {str(e)}")
            return []
    
    async def _extract_from_sidebar(self, base_url: str) -> List[str]:
        """Try to extract URLs that might be in sidebar structure"""
        try:
            # Common Docusaurus patterns for documentation
            patterns = [
                '/docs/intro',
                '/docs/tutorial-basics/create-a-document',
                '/docs/tutorial-basics/create-a-page',
                '/docs/tutorial-basics/create-a-blog-post',
                '/docs/tutorial-basics/markdown-features',
                '/docs/tutorial-basics/congratulations',
                '/docs/modules/ros2/intro',
                '/docs/modules/ros2/chapter-1/introduction-to-ros2-nervous-system',
                '/docs/modules/ros2/chapter-2/communicating-with-robots-using-ros2',
                '/docs/modules/ros2/chapter-3/humanoid-robot-structure-with-urdf',
                '/docs/modules/gazebo-unity/intro',
                '/docs/modules/gazebo-unity/chapter-1/physics-simulation-with-gazebo',
                '/docs/modules/gazebo-unity/chapter-2/high-fidelity-interaction-with-unity',
                '/docs/modules/gazebo-unity/chapter-3/simulated-sensors-for-perception',
                '/docs/modules/isaac/intro',
                '/docs/modules/isaac/chapter-1/nvidia-isaac-sim-synthetic-data',
                '/docs/modules/isaac/chapter-2/isaac-ros-perception-localization',
                '/docs/modules/isaac/chapter-3/nav2-humanoid-navigation',
                '/docs/modules/vla/intro',
                '/docs/modules/vla/chapter-1/voice-to-action-with-speech-models',
                '/docs/modules/vla/chapter-2/language-driven-cognitive-planning',
                '/docs/modules/vla/chapter-3/capstone-autonomous-humanoid'
            ]
            
            return [urljoin(base_url, pattern) for pattern in patterns]
        except Exception as e:
            logger.warning(f"Could not extract from sidebar patterns: {str(e)}")
            return []
    
    async def _extract_from_patterns(self, base_url: str) -> List[str]:
        """Extract URLs using common Docusaurus patterns"""
        try:
            # Try to access the sidebars.js or similar files to understand the structure
            # For now, we'll use the known structure from your book
            base_patterns = [
                "/docs/intro",
                "/docs/modules/ros2/intro",
                "/docs/modules/gazebo-unity/intro", 
                "/docs/modules/isaac/intro",
                "/docs/modules/vla/intro"
            ]
            
            chapter_patterns = [
                "/docs/modules/ros2/chapter-1/introduction-to-ros2-nervous-system",
                "/docs/modules/ros2/chapter-2/communicating-with-robots-using-ros2",
                "/docs/modules/ros2/chapter-3/humanoid-robot-structure-with-urdf",
                "/docs/modules/gazebo-unity/chapter-1/physics-simulation-with-gazebo",
                "/docs/modules/gazebo-unity/chapter-2/high-fidelity-interaction-with-unity",
                "/docs/modules/gazebo-unity/chapter-3/simulated-sensors-for-perception",
                "/docs/modules/isaac/chapter-1/nvidia-isaac-sim-synthetic-data",
                "/docs/modules/isaac/chapter-2/isaac-ros-perception-localization",
                "/docs/modules/isaac/chapter-3/nav2-humanoid-navigation",
                "/docs/modules/vla/chapter-1/voice-to-action-with-speech-models",
                "/docs/modules/vla/chapter-2/language-driven-cognitive-planning",
                "/docs/modules/vla/chapter-3/capstone-autonomous-humanoid"
            ]
            
            all_patterns = base_patterns + chapter_patterns
            return [urljoin(base_url, pattern) for pattern in all_patterns]
        except Exception as e:
            logger.warning(f"Could not extract from patterns: {str(e)}")
            return []
    
    def _is_same_domain(self, url: str, base_url: str) -> bool:
        """Check if a URL is from the same domain as the base URL"""
        return urlparse(url).netloc == urlparse(base_url).netloc
    
    async def extract_content_from_url(self, url: str) -> Dict[str, Any]:
        """Extract content from a single URL"""
        logger.info(f"Extracting content from {url}")
        
        try:
            response = requests.get(url, timeout=30)
            response.raise_for_status()
            
            soup = BeautifulSoup(response.text, 'html.parser')
            
            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()
            
            # Extract title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else ""
            
            # Extract main content - for Docusaurus sites, this is typically in main content areas
            # Look for common Docusaurus content containers
            main_content = (soup.find('main') or 
                          soup.find('article') or 
                          soup.find('div', class_='container') or
                          soup.find('div', {'id': 'main-content'}) or
                          soup.find('div', class_='docItemContainer'))
            
            if main_content:
                content_text = main_content.get_text(separator=' ', strip=True)
            else:
                content_text = soup.get_text(separator=' ', strip=True)
            
            # Clean up content
            content_text = ' '.join(content_text.split())
            
            return {
                'url': url,
                'title': title,
                'content': content_text
            }
        except Exception as e:
            logger.error(f"Failed to extract content from {url}: {str(e)}")
            return {}
    
    def chunk_content(self, content: str, max_chunk_size: int = 1000, overlap: int = 200) -> List[str]:
        """Chunk content into smaller pieces"""
        if len(content) <= max_chunk_size:
            return [content]
        
        chunks = []
        start = 0
        
        while start < len(content):
            end = start + max_chunk_size
            
            # If we're not at the end, try to break at a sentence boundary
            if end < len(content):
                # Look for sentence endings near the end
                sentence_end = -1
                for i in range(end, max(0, end - 200), -1):
                    if content[i] in '.!?':
                        sentence_end = i + 1
                        break
                
                if sentence_end != -1 and sentence_end > start + 500:  # Don't make chunks too small
                    end = sentence_end
                else:
                    # If no sentence end found, break at word boundary
                    for i in range(end, max(0, end - 200), -1):
                        if content[i] == ' ':
                            end = i
                            break
            
            chunk = content[start:end].strip()
            if chunk:
                chunks.append(chunk)
            
            start = end - overlap if end < len(content) else end
        
        logger.info(f"Content chunked into {len(chunks)} pieces")
        return chunks
    
    async def store_content_in_qdrant(self, content_data: Dict[str, Any]):
        """Store content in Qdrant with a mock embedding (for testing)"""
        if not content_data or not content_data.get('content'):
            return
        
        # Chunk the content
        content_chunks = self.chunk_content(content_data['content'])
        
        points = []
        for i, chunk in enumerate(content_chunks):
            # Create a mock embedding (in real implementation, this would be generated by an embedding model)
            # For now, we'll create a simple mock embedding for testing
            mock_embedding = [i * 0.01 for i in range(1536)]  # 1536-dimensional vector
            
            # Create a unique ID for the chunk
            chunk_id = hashlib.md5(f"{content_data['url']}_{i}".encode()).hexdigest()
            
            # Create payload with metadata
            payload = {
                'url': content_data['url'],
                'title': content_data['title'],
                'content': chunk,
                'chunk_index': i
            }
            
            # Add point to collection
            points.append(
                models.PointStruct(
                    id=chunk_id,
                    vector=mock_embedding,
                    payload=payload
                )
            )
        
        if points:
            # Upload points to Qdrant
            await self.qdrant_client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logger.info(f"Successfully stored {len(points)} vectors in Qdrant for {content_data['url']}")
        else:
            logger.warning(f"No points to store for {content_data['url']}")
    
    async def run_ingestion(self, base_url: str):
        """Run the comprehensive ingestion pipeline"""
        logger.info("Starting comprehensive ingestion pipeline...")
        
        await self.create_collection_if_not_exists()
        
        # Discover ALL URLs
        urls = await self.discover_all_urls(base_url)
        
        total_processed = 0
        total_chunks = 0
        total_urls = len(urls)
        
        logger.info(f"Starting to process {total_urls} URLs...")
        
        for i, url in enumerate(urls):
            logger.info(f"Processing URL {i+1}/{total_urls}: {url}")
            
            if url in self.visited_urls:
                logger.info(f"Already visited {url}, skipping...")
                continue
            
            self.visited_urls.add(url)
            
            # Extract content
            content_data = await self.extract_content_from_url(url)
            
            if content_data and content_data.get('content'):
                # Store in Qdrant
                await self.store_content_in_qdrant(content_data)
                total_processed += 1
                
                # Count chunks
                content_chunks = self.chunk_content(content_data['content'])
                total_chunks += len(content_chunks)
                
                logger.info(f"Successfully processed {url} with {len(content_chunks)} chunks")
            else:
                logger.warning(f"No content extracted from {url}")
        
        logger.info(f"Comprehensive ingestion pipeline completed!")
        logger.info(f"Summary: Processed {total_processed} out of {total_urls} URLs")
        logger.info(f"Total content chunks stored: {total_chunks}")


async def main():
    """Main function to run the comprehensive ingestion"""
    logger.info("Starting comprehensive book content ingestion...")
    
    # Use the deployed book URL
    book_url = "https://hackathon1-book-ragchatbot.vercel.app"
    
    # Initialize the ingestor
    ingestor = ComprehensiveBookIngestor()
    
    # Run comprehensive ingestion
    await ingestor.run_ingestion(book_url)
    
    logger.info("Comprehensive book content ingestion completed!")


if __name__ == "__main__":
    asyncio.run(main())