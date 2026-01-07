"""
FastAPI backend for the RAG chatbot
"""
from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import uvicorn
import asyncio
import logging
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import after loading environment variables
from api.rag_service import RAGService

# Initialize FastAPI app
app = FastAPI(
    title="Book RAG Chatbot API",
    description="API for the Retrieval-Augmented Generation chatbot using book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize RAG service (deferred until needed)
rag_service = None

def get_rag_service():
    global rag_service
    if rag_service is None:
        rag_service = RAGService()
    return rag_service

# Pydantic models
class Message(BaseModel):
    role: str  # "user" or "assistant"
    content: str
    timestamp: Optional[float] = None

class ChatRequest(BaseModel):
    message: str
    session_id: Optional[str] = None
    history: Optional[List[Message]] = []

class ChatResponse(BaseModel):
    response: str
    session_id: str
    sources: Optional[List[Dict[str, Any]]] = []

class QueryRequest(BaseModel):
    query: str
    top_k: Optional[int] = 5

class QueryResponse(BaseModel):
    results: List[Dict[str, Any]]

# In-memory storage for sessions (in production, use a database)
chat_sessions: Dict[str, List[Message]] = {}

@app.get("/")
async def root():
    return {"message": "Book RAG Chatbot API is running!"}

@app.get("/health")
async def health_check():
    return {"status": "healthy"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Main chat endpoint that processes user messages and returns AI responses
    """
    try:
        # Get the RAG service (initialize if needed)
        service = get_rag_service()

        # Generate or use provided session ID
        session_id = request.session_id or f"session_{len(chat_sessions)}"

        # Add user message to history
        user_message = Message(role="user", content=request.message)
        if session_id not in chat_sessions:
            chat_sessions[session_id] = []
        chat_sessions[session_id].append(user_message)

        # Process the query through RAG pipeline
        rag_result = await service.query_and_respond(request.message)
        response_text = rag_result["response"]
        sources = rag_result["sources"]

        # Add AI response to history
        ai_message = Message(role="assistant", content=response_text)
        chat_sessions[session_id].append(ai_message)

        # Return response with sources
        response = ChatResponse(
            response=response_text,
            session_id=session_id,
            sources=sources
        )

        return response
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Query endpoint for retrieving relevant documents from the vector database
    """
    try:
        # Get the RAG service (initialize if needed)
        service = get_rag_service()

        # Use the RAG service to search for documents
        documents = await service.search(request.query, request.top_k)

        results = [
            {
                "id": doc.id,
                "content": doc.content,
                "url": doc.url,
                "title": doc.title,
                "score": doc.score
            }
            for doc in documents
        ]

        return QueryResponse(results=results)
    except Exception as e:
        logger.error(f"Error in query endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/sessions/{session_id}")
async def get_session(session_id: str):
    """
    Retrieve chat session history
    """
    if session_id not in chat_sessions:
        raise HTTPException(status_code=404, detail="Session not found")
    
    return {"session_id": session_id, "messages": chat_sessions[session_id]}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)