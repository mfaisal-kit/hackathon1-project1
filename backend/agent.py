"""
Mock RAG Agent with OpenAI Agents SDK Pattern

This module implements a mock version of the RAG-enabled agent that simulates
the OpenAI Agents SDK behavior without requiring API keys.
"""
import os
import asyncio
import logging
from typing import Dict, Any, List
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MockFunctionTool:
    """
    Mock function tool decorator that simulates the @function_tool behavior
    """
    def __init__(self, func):
        self.func = func
        self.__name__ = func.__name__
        self.__doc__ = func.__doc__

    def __call__(self, *args, **kwargs):
        return self.func(*args, **kwargs)

def mock_function_tool(func):
    """
    Mock function tool decorator
    """
    return MockFunctionTool(func)

class MockAgent:
    """
    Mock Agent class that simulates the Agent behavior
    """
    def __init__(self, name, instructions, tools=None):
        self.name = name
        self.instructions = instructions
        self.tools = tools or []

class MockRunner:
    """
    Mock Runner class that simulates the Runner behavior
    """
    @staticmethod
    async def run(agent, user_message):
        """
        Mock run method that simulates the agent behavior
        """
        # Simulate processing the user message with tools
        result = f"Mock response to: {user_message}\n\n"
        
        # Simulate using tools if they exist
        if agent.tools:
            result += "Using tools to retrieve information:\n"
            for tool in agent.tools:
                if hasattr(tool, '__name__') and tool.__name__ == 'retrieve_book_content':
                    # Simulate calling the retrieve_book_content tool
                    mock_result = await retrieve_book_content(user_message)
                    result += f"- Retrieved {mock_result.get('total_results', 0)} relevant documents\n"
                    result += f"- Sources include: Introduction to Physical AI, ROS 2 Introduction\n\n"
        
        result += "Based on the retrieved information, here's what I found about your query."
        
        class MockResult:
            def __init__(self, output):
                self.final_output = output
        
        return MockResult(result)

@mock_function_tool
async def retrieve_book_content(query: str, top_k: int = 5) -> Dict[str, Any]:
    """
    Retrieve relevant content from the Physical AI & Humanoid Robotics course book.
    
    Args:
        query: The search query to find relevant book content
        top_k: Number of results to retrieve (default 5)
    """
    try:
        # Return mock results to simulate the search
        mock_results = [
            {
                "content": f"Sample content related to '{query}' from the Physical AI & Humanoid Robotics course. This covers fundamental concepts about {query} in the context of robotics and AI systems.",
                "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/intro",
                "title": "Introduction to Physical AI",
                "score": 0.9
            },
            {
                "content": f"More information about {query} in the context of robotics. This section discusses how {query} is implemented in real-world robotic systems and its importance in the field.",
                "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/intro",
                "title": "ROS 2 Introduction",
                "score": 0.8
            },
            {
                "content": f"Advanced concepts about {query} and its applications. This includes practical examples and use cases for {query} in humanoid robotics and AI systems.",
                "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/isaac/intro",
                "title": "NVIDIA Isaac Introduction",
                "score": 0.7
            }
        ]
        
        # Limit to top_k results
        limited_results = mock_results[:top_k]
        
        return {
            "query": query,
            "results": limited_results,
            "total_results": len(limited_results)
        }
    except Exception as e:
        logger.error(f"Error in retrieve_book_content: {str(e)}")
        return {"error": f"Retrieval failed: {str(e)}"}

class RAGAgent:
    """
    RAG Agent that simulates OpenAI Agents SDK behavior without API requirements
    """
    
    def __init__(self):
        """
        Initialize the RAG Agent with mock components
        """
        # Create the mock agent with instructions specific to the book content
        self.agent = MockAgent(
            name="Book RAG Assistant",
            instructions="""
            You are an AI assistant for the Physical AI & Humanoid Robotics course.
            Your responses must be grounded only in the book content provided through the retrieval tool.
            When answering questions:
            1. Use only the information from the retrieved documents
            2. Clearly cite sources from the retrieved content
            3. If the retrieved content doesn't answer the question, say so
            4. Maintain a helpful and educational tone
            5. Provide accurate information based on the book content
            """,
            tools=[retrieve_book_content]
        )
        
    async def chat(self, user_message: str) -> Dict[str, Any]:
        """
        Complete chat flow: run agent with user message and get response
        """
        try:
            # Simulate running the agent with the user message
            result = await MockRunner.run(self.agent, user_message)
            response = result.final_output
            
            return {
                "response": response,
                "success": True
            }
        except Exception as e:
            logger.error(f"Error in chat: {str(e)}")
            return {
                "response": "I'm sorry, but I encountered an error while processing your request.",
                "success": False,
                "error": str(e)
            }
    
    async def validate_response_grounding(self, query: str, response: str) -> Dict[str, Any]:
        """
        Mock validation that simulates grounding check
        """
        # For mock purposes, we'll simulate a successful validation
        return {
            "query": query,
            "grounded_in_retrieved_content": True,  # Simulate successful grounding
            "relevant_sources_count": 2,  # Simulate 2 sources
            "relevant_sources": [
                {
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/intro",
                    "title": "Introduction to Physical AI",
                    "content_preview": f"Content related to {query}..."
                },
                {
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/intro",
                    "title": "ROS 2 Introduction", 
                    "content_preview": f"More information about {query}..."
                }
            ],
            "validation_performed": True
        }


class RAGAgentTester:
    """
    Class to test the RAG Agent functionality
    """
    
    def __init__(self, rag_agent: RAGAgent):
        self.rag_agent = rag_agent
        self.test_queries = [
            "What is ROS 2?",
            "Explain humanoid robotics",
            "How does Gazebo simulation work?",
            "What is NVIDIA Isaac?",
            "Tell me about Vision-Language-Action systems"
        ]
    
    async def run_agent_test(self) -> Dict[str, Any]:
        """
        Run comprehensive test of the RAG agent
        """
        logger.info("Starting RAG agent test...")
        
        test_results = {
            "total_queries": len(self.test_queries),
            "successful_responses": 0,
            "failed_responses": 0,
            "responses_with_grounding": 0,
            "responses_without_grounding": 0,
            "query_results": []
        }
        
        for i, query in enumerate(self.test_queries):
            logger.info(f"Testing query {i+1}/{len(self.test_queries)}: {query}")
            
            # Get response from agent
            result = await self.rag_agent.chat(query)
            
            if result.get("success"):
                test_results["successful_responses"] += 1
                
                # Validate response grounding
                validation = await self.rag_agent.validate_response_grounding(query, result["response"])
                
                if validation.get("grounded_in_retrieved_content"):
                    test_results["responses_with_grounding"] += 1
                else:
                    test_results["responses_without_grounding"] += 1
                
                test_results["query_results"].append({
                    "query": query,
                    "response": result["response"],
                    "grounding_validation": validation,
                    "success": True
                })
            else:
                test_results["failed_responses"] += 1
                test_results["query_results"].append({
                    "query": query,
                    "response": "Failed to get response",
                    "success": False
                })
        
        return test_results
    
    async def generate_test_report(self) -> str:
        """
        Generate a comprehensive test report
        """
        test_results = await self.run_agent_test()
        
        report = []
        report.append("=" * 60)
        report.append("RAG AGENT TEST REPORT")
        report.append("=" * 60)
        report.append(f"Total Queries: {test_results['total_queries']}")
        report.append(f"Successful Responses: {test_results['successful_responses']}")
        report.append(f"Failed Responses: {test_results['failed_responses']}")
        report.append(f"Responses with Grounding: {test_results['responses_with_grounding']}")
        report.append(f"Responses without Grounding: {test_results['responses_without_grounding']}")
        report.append("")
        
        # Individual Query Results
        report.append("INDIVIDUAL QUERY RESULTS:")
        for i, result in enumerate(test_results['query_results']):
            report.append(f"  Query {i+1}: {result['query']}")
            report.append(f"    Success: {result.get('success', False)}")
            
            if result.get('success'):
                grounding = result.get('grounding_validation', {})
                report.append(f"    Grounded: {grounding.get('grounded_in_retrieved_content', False)}")
                report.append(f"    Sources: {grounding.get('relevant_sources_count', 0)}")
            
            report.append(f"    Response Preview: {result.get('response', 'No response')[:100]}...")
            report.append("")
        
        report.append("=" * 60)
        report.append("TEST COMPLETED")
        report.append("=" * 60)
        
        return "\n".join(report)


async def main():
    """
    Main function to demonstrate the RAG Agent
    """
    logger.info("Initializing RAG Agent with OpenAI Agents SDK pattern...")
    
    # Initialize the RAG Agent
    rag_agent = RAGAgent()
    
    # Create tester
    tester = RAGAgentTester(rag_agent)
    
    # Run comprehensive test
    test_report = await tester.generate_test_report()
    print(test_report)
    
    # Example of single query
    logger.info("\nTesting single query...")
    sample_query = "What is the robotic nervous system?"
    result = await rag_agent.chat(sample_query)
    
    print(f"\nQuery: {sample_query}")
    print(f"Response: {result.get('response', 'No response')}")
    
    if result.get('success'):
        # Validate grounding
        validation = await rag_agent.validate_response_grounding(sample_query, result['response'])
        print(f"Grounded in book content: {validation.get('grounded_in_retrieved_content', False)}")
        print(f"Relevant sources: {validation.get('relevant_sources_count', 0)}")


if __name__ == "__main__":
    asyncio.run(main())