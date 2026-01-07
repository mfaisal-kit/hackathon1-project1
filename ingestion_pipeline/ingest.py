"""
Website Ingestion Pipeline for RAG Chatbot

This script handles:
1. Discovering and fetching content from the deployed Docusaurus website
2. Extracting and chunking content
3. Generating embeddings
4. Storing vectors in Qdrant Cloud
"""
import os
import asyncio
import hashlib
from typing import List, Dict, Any
from urllib.parse import urljoin, urlparse
import requests
from bs4 import BeautifulSoup
from openai import AsyncOpenAI
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WebsiteIngestor:
    def __init__(self, base_url: str, qdrant_url: str, qdrant_api_key: str, openai_api_key: str):
        self.base_url = base_url
        self.qdrant_client = AsyncQdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key
        )
        self.openai_client = AsyncOpenAI(api_key=openai_api_key)
        self.visited_urls = set()
        
    async def discover_urls(self) -> List[str]:
        """Discover all URLs from the deployed Docusaurus website"""
        logger.info(f"Discovering URLs from {self.base_url}")
        urls = []
        
        # Get the main page
        response = await self._fetch_page(self.base_url)
        if response:
            urls.extend(await self._extract_links(response, self.base_url))
        
        # For a deployed Docusaurus site, we can also use the sitemap if available
        sitemap_url = urljoin(self.base_url, "sitemap.xml")
        sitemap_urls = await self._extract_from_sitemap(sitemap_url)
        urls.extend(sitemap_urls)
        
        # Remove duplicates and filter to only include URLs from the same domain
        unique_urls = list(set(urls))
        filtered_urls = [url for url in unique_urls 
                        if self._is_same_domain(url, self.base_url) and not url.endswith(('.jpg', '.jpeg', '.png', '.gif', '.pdf', '.zip'))]
        
        logger.info(f"Discovered {len(filtered_urls)} URLs")
        return filtered_urls
    
    async def _fetch_page(self, url: str) -> str:
        """Fetch a single page and return its content"""
        try:
            response = requests.get(url, timeout=30)
            response.raise_for_status()
            return response.text
        except Exception as e:
            logger.error(f"Failed to fetch {url}: {str(e)}")
            return ""
    
    async def _extract_links(self, html_content: str, base_url: str) -> List[str]:
        """Extract all links from HTML content"""
        soup = BeautifulSoup(html_content, 'html.parser')
        links = []
        
        for link in soup.find_all('a', href=True):
            href = link['href']
            full_url = urljoin(base_url, href)
            
            # Only include internal links
            if self._is_same_domain(full_url, base_url):
                links.append(full_url)
        
        return links
    
    async def _extract_from_sitemap(self, sitemap_url: str) -> List[str]:
        """Extract URLs from sitemap if available"""
        try:
            response = requests.get(sitemap_url, timeout=30)
            if response.status_code == 200:
                soup = BeautifulSoup(response.content, 'xml')
                urls = []
                for loc in soup.find_all('loc'):
                    urls.append(loc.text.strip())
                logger.info(f"Found {len(urls)} URLs from sitemap")
                return urls
        except Exception as e:
            logger.warning(f"Could not fetch sitemap: {str(e)}")
        
        return []
    
    def _is_same_domain(self, url: str, base_url: str) -> bool:
        """Check if a URL is from the same domain as the base URL"""
        return urlparse(url).netloc == urlparse(base_url).netloc
    
    async def extract_content(self, url: str) -> Dict[str, Any]:
        """Extract content from a single URL"""
        logger.info(f"Extracting content from {url}")
        
        html_content = await self._fetch_page(url)
        if not html_content:
            return {}
        
        soup = BeautifulSoup(html_content, 'html.parser')
        
        # Remove script and style elements
        for script in soup(["script", "style"]):
            script.decompose()
        
        # Extract title
        title_tag = soup.find('title')
        title = title_tag.get_text().strip() if title_tag else ""
        
        # Extract main content - for Docusaurus sites, this is typically in main content areas
        main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='container')
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
    
    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using OpenAI API"""
        try:
            response = await self.openai_client.embeddings.create(
                input=text,
                model="text-embedding-3-small"  # Using a compatible model
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Failed to generate embedding for text: {str(e)}")
            return []
    
    async def store_in_qdrant(self, chunks: List[Dict[str, Any]], collection_name: str = "book_content"):
        """Store chunks with embeddings in Qdrant"""
        logger.info(f"Storing {len(chunks)} chunks in Qdrant collection '{collection_name}'")
        
        # Create collection if it doesn't exist
        try:
            await self.qdrant_client.get_collection(collection_name)
        except:
            # Collection doesn't exist, create it
            await self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # text-embedding-3-small returns 1536-dim vectors
            )
            logger.info(f"Created new Qdrant collection: {collection_name}")
        
        points = []
        for i, chunk_data in enumerate(chunks):
            text = chunk_data['content']
            url = chunk_data['url']
            title = chunk_data.get('title', '')
            
            # Generate embedding
            embedding = await self.generate_embedding(text)
            if not embedding:
                continue
            
            # Create a unique ID for the chunk
            chunk_id = hashlib.md5(f"{url}_{i}".encode()).hexdigest()
            
            # Create payload with metadata
            payload = {
                'url': url,
                'title': title,
                'content': text,
                'chunk_index': i
            }
            
            # Add point to collection
            points.append(
                models.PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload=payload
                )
            )
        
        if points:
            # Upload points to Qdrant
            await self.qdrant_client.upsert(
                collection_name=collection_name,
                points=points
            )
            logger.info(f"Successfully stored {len(points)} vectors in Qdrant")
        else:
            logger.warning("No points to store in Qdrant")
    
    async def process_url(self, url: str) -> List[Dict[str, Any]]:
        """Process a single URL: extract, chunk, and prepare for storage"""
        if url in self.visited_urls:
            return []
        
        self.visited_urls.add(url)
        
        # Extract content
        content_data = await self.extract_content(url)
        if not content_data or not content_data.get('content'):
            return []
        
        # Chunk content
        content_chunks = self.chunk_content(content_data['content'])
        
        # Prepare chunks with metadata
        chunked_data = []
        for i, chunk in enumerate(content_chunks):
            chunked_data.append({
                'url': content_data['url'],
                'title': content_data['title'],
                'content': chunk,
                'chunk_index': i
            })
        
        return chunked_data
    
    async def run_ingestion(self, collection_name: str = "book_content"):
        """Run the full ingestion pipeline"""
        logger.info("Starting ingestion pipeline")
        
        # Discover URLs
        urls = await self.discover_urls()
        
        all_chunks = []
        
        # Process each URL
        for i, url in enumerate(urls):
            logger.info(f"Processing URL {i+1}/{len(urls)}: {url}")
            chunks = await self.process_url(url)
            all_chunks.extend(chunks)
        
        # Store in Qdrant
        await self.store_in_qdrant(all_chunks, collection_name)
        
        logger.info(f"Ingestion complete. Processed {len(all_chunks)} content chunks from {len(urls)} URLs")


async def main():
    # Configuration from environment variables
    base_url = os.getenv("BASE_URL", "https://hackathon1-book-ragchatbot.vercel.app")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    openai_api_key = os.getenv("OPENAI_API_KEY")
    
    if not all([qdrant_url, qdrant_api_key, openai_api_key]):
        raise ValueError("Missing required environment variables: QDRANT_URL, QDRANT_API_KEY, OPENAI_API_KEY")
    
    # Initialize the ingestor
    ingestor = WebsiteIngestor(
        base_url=base_url,
        qdrant_url=qdrant_url,
        qdrant_api_key=qdrant_api_key,
        openai_api_key=openai_api_key
    )
    
    # Run the ingestion pipeline
    await ingestor.run_ingestion()


if __name__ == "__main__":
    asyncio.run(main())