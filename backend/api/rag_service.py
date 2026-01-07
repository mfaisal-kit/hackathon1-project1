"""
RAG Service Layer
Handles the retrieval-augmented generation logic by integrating with the ingestion pipeline
"""
import os
import logging
from typing import List, Dict, Any, Optional
from openai import AsyncOpenAI
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from pydantic import BaseModel
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Document(BaseModel):
    id: str
    content: str
    url: str
    title: str
    score: float

class RAGService:
    def __init__(self):
        # Initialize clients
        self.qdrant_client = AsyncQdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.openai_client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.collection_name = "book_content"
    
    async def search(self, query: str, top_k: int = 5) -> List[Document]:
        """
        Search for relevant documents in the vector database
        """
        try:
            # Generate embedding for the query
            query_embedding = await self._generate_embedding(query)
            
            # Search in Qdrant
            search_result = await self.qdrant_client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                with_payload=True
            )
            
            # Convert results to Document objects
            documents = []
            for result in search_result:
                documents.append(
                    Document(
                        id=result.id,
                        content=result.payload.get('content', ''),
                        url=result.payload.get('url', ''),
                        title=result.payload.get('title', ''),
                        score=result.score
                    )
                )
            
            logger.info(f"Found {len(documents)} relevant documents for query: {query[:50]}...")
            return documents
        except Exception as e:
            logger.error(f"Error searching documents: {str(e)}")
            return []
    
    async def _generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using OpenAI API
        """
        try:
            response = await self.openai_client.embeddings.create(
                input=text,
                model="text-embedding-3-small"
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {str(e)}")
            return []
    
    async def generate_response(self, query: str, context_docs: List[Document]) -> str:
        """
        Generate a response using the query and retrieved context
        """
        try:
            # Construct context from retrieved documents
            context = "\n\n".join([doc.content for doc in context_docs])
            
            # Create a prompt with the query and context
            prompt = f"""
            You are an AI assistant for the Physical AI & Humanoid Robotics course.
            Use the following context to answer the user's question.
            If the context doesn't contain enough information, say so.
            
            Context:
            {context}
            
            Question: {query}
            
            Answer:
            """
            
            # Generate response using OpenAI
            response = await self.openai_client.chat.completions.create(
                model="gpt-3.5-turbo",  # or gpt-4 if preferred
                messages=[
                    {"role": "system", "content": "You are a helpful assistant for the Physical AI & Humanoid Robotics course. Answer questions based on the provided context."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=500,
                temperature=0.3
            )
            
            return response.choices[0].message.content.strip()
        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return "I encountered an error while generating a response. Please try again later."
    
    async def query_and_respond(self, query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Complete RAG flow: query documents and generate response
        """
        # Search for relevant documents
        documents = await self.search(query, top_k)
        
        # Generate response based on retrieved documents
        response = await self.generate_response(query, documents)
        
        # Prepare result with sources
        sources = [
            {
                "id": doc.id,
                "content": doc.content[:200] + "..." if len(doc.content) > 200 else doc.content,
                "url": doc.url,
                "title": doc.title,
                "score": doc.score
            }
            for doc in documents
        ]
        
        return {
            "response": response,
            "sources": sources
        }