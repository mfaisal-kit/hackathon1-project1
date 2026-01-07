"""
FastAPI Backend for RAG Agent Integration

This module implements a FastAPI backend that exposes the RAG agent
through API endpoints for frontend integration.
"""
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import asyncio
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Book RAG Chatbot API",
    description="API for the Retrieval-Augmented Generation chatbot using book content",
    version="1.0.0"
)

# Add CORS middleware to allow frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend origin
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models for request/response validation
class QueryRequest(BaseModel):
    query: str
    session_id: Optional[str] = None

class QueryResponse(BaseModel):
    response: str
    session_id: str
    sources: List[Dict[str, Any]]
    success: bool

class HealthResponse(BaseModel):
    status: str

# In-memory storage for sessions (in production, use a database)
chat_sessions: Dict[str, List[Dict[str, str]]] = {}

# Mock RAG agent service (in real implementation, this would connect to the actual agent)
class MockRAGAgentService:
    """
    Mock RAG agent service that simulates the agent behavior
    In a real implementation, this would connect to the actual RAG agent from Spec 3
    """
    
    async def query(self, query_text: str, session_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Process a query through the RAG agent
        """
        # Simulate processing time
        await asyncio.sleep(0.5)

        # Generate different responses based on keywords in the query
        query_lower = query_text.lower()

        # Define more specific keyword categories with better precedence
        # Check for more specific topics first to avoid generic matches

        # Check for specific terms first, in order of specificity

        # Check for humanoid robotics specifically
        if 'humanoid' in query_lower and 'robotics' in query_lower:
            response_text = f"Based on the book content, humanoid robotics involves creating robots with human-like characteristics and capabilities. These robots have anthropomorphic structures with legs, arms, and sometimes faces.\n\n"
            response_text += "Humanoid robots are designed to operate in human environments and interact with humans naturally. The course covers how AI systems control these robots, including locomotion, manipulation, and interaction capabilities.\n\n"
            response_text += "Key challenges include balance control, bipedal walking, and human-robot interaction that mimics human social behaviors."

            sources = [
                {
                    "title": "Introduction to Physical AI",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/intro",
                    "content_preview": f"Fundamentals of physical AI and embodied intelligence..."
                },
                {
                    "title": "Humanoid Robot Structure with URDF",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/chapter-3/humanoid-robot-structure-with-urdf",
                    "content_preview": f"URDF files for humanoid robot description..."
                }
            ]
        # Check for ROS/ROS2 specifically
        elif any(keyword in query_lower for keyword in ['ros ', 'ros2', 'robot operating system']):
            response_text = f"Based on the book content, ROS 2 (Robot Operating System 2) is the middleware that connects AI decision-making to physical humanoid robot control. It provides a communication infrastructure for distributed robotic systems.\n\n"
            response_text += "ROS 2 features include improved security, real-time support, and better scalability compared to ROS 1. It's essential for creating distributed robotic applications where different nodes communicate via topics, services, and actions.\n\n"
            response_text += "For humanoid robotics, ROS 2 provides the communication backbone that allows AI systems to coordinate with various robot subsystems like perception, navigation, and manipulation."

            sources = [
                {
                    "title": "ROS 2 Introduction",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/intro",
                    "content_preview": f"Information about ROS 2 as the robotic nervous system..."
                },
                {
                    "title": "Communicating with Robots Using ROS 2",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/chapter-2/communicating-with-robots-using-ros2",
                    "content_preview": f"How AI agents interact with robots via rclpy..."
                }
            ]
        # Check for general robotics (but avoid ROS matches)
        elif 'robotics' in query_lower and not any(keyword in query_lower for keyword in ['humanoid', 'ros ', 'ros2']):
            response_text = f"Based on the book content, robotics is an interdisciplinary field that combines engineering and computer science to design, construct, operate, and use robots. The course covers how AI systems connect to and control physical robots.\n\n"
            response_text += "Robotics involves several key areas: mechanical engineering for robot structure, electrical engineering for power and control systems, and computer science for intelligence and autonomy. The course focuses on how AI algorithms enable robots to perceive their environment, make decisions, and execute actions.\n\n"
            response_text += "In the context of this course, robotics specifically addresses the integration of AI with physical systems to create intelligent machines that can operate in the real world."

            sources = [
                {
                    "title": "Introduction to Physical AI",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/intro",
                    "content_preview": f"Overview of robotics and AI integration..."
                },
                {
                    "title": "The Robotic Nervous System (ROS 2)",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/intro",
                    "content_preview": f"ROS 2 as the middleware for robotics..."
                }
            ]
        # Check for physical AI specifically
        elif 'physical ai' in query_lower or ('physical' in query_lower and 'ai' in query_lower):
            response_text = f"Based on the book content, Physical AI refers to the field that connects artificial intelligence with physical robot control. Unlike traditional AI that operates in digital spaces, Physical AI focuses on how AI systems interact with and control physical robots in the real world.\n\n"
            response_text += "Physical AI involves several key challenges: perception (understanding the physical environment), planning (deciding what actions to take), and control (executing those actions with physical actuators). The course covers how to bridge the gap between high-level AI decision-making and low-level robot control.\n\n"
            response_text += "The field of Physical AI aims to create AI systems that can effectively manipulate and interact with the physical world through robotic embodiments."

            sources = [
                {
                    "title": "Introduction to Physical AI",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/intro",
                    "content_preview": f"Definition and overview of Physical AI..."
                },
                {
                    "title": "ROS 2 as the Robotic Nervous System",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/chapter-1/introduction-to-ros2-nervous-system",
                    "content_preview": f"Physical AI and the robotic nervous system..."
                }
            ]
        # Check for Gazebo/simulation
        elif any(keyword in query_lower for keyword in ['gazebo', 'simulation', 'physics']):
            response_text = f"Based on the book content, Gazebo is a physics simulation environment used extensively in robotics development. It provides realistic physics simulation that allows testing robotic systems in virtual environments that closely approximate real-world conditions.\n\n"
            response_text += "Gazebo includes features like accurate modeling of forces, collisions, and sensor data. It's part of the digital twin concept where physical robots have virtual counterparts for testing.\n\n"
            response_text += "For humanoid robotics, Gazebo enables safe testing of complex movements and interactions before deployment to real hardware."

            sources = [
                {
                    "title": "Physics Simulation with Gazebo",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/gazebo-unity/chapter-1/physics-simulation-with-gazebo",
                    "content_preview": f"Physics simulation for robotic systems..."
                },
                {
                    "title": "Simulated Sensors for Perception",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/gazebo-unity/chapter-3/simulated-sensors-for-perception",
                    "content_preview": f"Simulating camera, LiDAR, and other sensors..."
                }
            ]
        # Check for Isaac/NVIDIA
        elif any(keyword in query_lower for keyword in ['isaac', 'nvidia', 'navigation']):
            response_text = f"Based on the book content, NVIDIA Isaac is a comprehensive platform for developing AI-powered robotic applications. Isaac Sim provides a photorealistic simulation environment for synthetic data generation.\n\n"
            response_text += "Isaac ROS bridges the gap between perception and action by providing accelerated algorithms for perception and localization. The platform integrates with Nav2 for navigation systems.\n\n"
            response_text += "For humanoid robotics, Isaac provides tools to develop sophisticated AI behaviors in virtual environments before transferring to physical robots."

            sources = [
                {
                    "title": "NVIDIA Isaac Sim & Synthetic Data",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/isaac/chapter-1/nvidia-isaac-sim-synthetic-data",
                    "content_preview": f"NVIDIA Isaac Sim for synthetic data generation..."
                },
                {
                    "title": "Nav2 for Humanoid Navigation",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/isaac/chapter-3/nav2-humanoid-navigation",
                    "content_preview": f"Navigation systems for humanoid robots..."
                }
            ]
        # Check for Vision-Language-Action
        elif any(keyword in query_lower for keyword in ['vision', 'language', 'action', 'vla']):
            response_text = f"Based on the book content, Vision-Language-Action (VLA) systems represent the cutting edge of AI robotics. These systems integrate visual perception, language understanding, and physical action in unified architectures.\n\n"
            response_text += "VLA systems enable robots to understand natural language commands and execute complex physical tasks. They bridge the gap between symbolic AI (language) and continuous control (action) through visual understanding.\n\n"
            response_text += "In humanoid robotics, VLA systems allow for more intuitive human-robot interaction through speech and gesture recognition."

            sources = [
                {
                    "title": "Voice-to-Action with Speech Models",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/vla/chapter-1/voice-to-action-with-speech-models",
                    "content_preview": f"Speech models for voice-to-action mapping..."
                },
                {
                    "title": "Language-Driven Cognitive Planning",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/vla/chapter-2/language-driven-cognitive-planning",
                    "content_preview": f"Language-driven planning for robotic tasks..."
                }
            ]
        else:
            # General response for other queries
            response_text = f"Based on the book content, here's what I found about '{query_text}':\n\n"
            response_text += "This topic relates to the intersection of artificial intelligence and physical robot control. The Physical AI & Humanoid Robotics course covers how AI systems connect to and control physical robots.\n\n"
            response_text += "Key concepts include the integration of perception, cognition, and action in embodied AI systems. The course explores various platforms and frameworks for developing AI-controlled robots."

            sources = [
                {
                    "title": "Introduction to Physical AI",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/intro",
                    "content_preview": f"Overview of physical AI concepts..."
                },
                {
                    "title": "Course Modules Overview",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/intro",
                    "content_preview": f"Course structure and module introductions..."
                }
            ]

        # Generate a session ID if not provided
        actual_session_id = session_id or f"session_{len(chat_sessions) + 1}"

        # Store the interaction in the session
        if actual_session_id not in chat_sessions:
            chat_sessions[actual_session_id] = []

        chat_sessions[actual_session_id].append({
            "query": query_text,
            "response": response_text
        })

        return {
            "response": response_text,
            "session_id": actual_session_id,
            "sources": sources,
            "success": True
        }

# Initialize the mock agent service
rag_agent_service = MockRAGAgentService()

@app.get("/")
async def root():
    return {"message": "Book RAG Chatbot API is running!"}

@app.get("/health", response_model=HealthResponse)
async def health_check():
    return HealthResponse(status="healthy")

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """
    Query endpoint that processes user queries through the RAG agent
    """
    try:
        logger.info(f"Processing query: {request.query}")
        
        # Process the query through the RAG agent
        result = await rag_agent_service.query(request.query, request.session_id)
        
        return QueryResponse(
            response=result["response"],
            session_id=result["session_id"],
            sources=result["sources"],
            success=result["success"]
        )
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/sessions/{session_id}")
async def get_session(session_id: str):
    """
    Retrieve chat session history
    """
    if session_id not in chat_sessions:
        raise HTTPException(status_code=404, detail="Session not found")
    
    return {"session_id": session_id, "messages": chat_sessions[session_id]}

# For local development
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)