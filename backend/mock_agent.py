"""
Mock RAG Agent with OpenAI Agents SDK

This module implements a mock version of the RAG-enabled agent using the OpenAI Agents SDK
that simulates the behavior without requiring OpenAI API calls, for testing when API quotas are exceeded.
"""
import os
import asyncio
import logging
from typing import Dict, Any, List
from dotenv import load_dotenv
from retrieve import RetrievalPipeline  # Import the retrieval pipeline from Spec 2

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MockRAGAgent:
    """
    Mock RAG Agent that simulates the OpenAI Agents SDK behavior without API calls
    """
    
    def __init__(self):
        """
        Initialize the Mock RAG Agent with retrieval pipeline
        """
        self.retrieval_pipeline = RetrievalPipeline()
        
        # Simulate agent instructions
        self.instructions = """
        You are an AI assistant for the Physical AI & Humanoid Robotics course.
        Your responses must be grounded only in the book content provided through the retrieval tool.
        When answering questions:
        1. Use only the information from the retrieved documents
        2. Clearly cite sources from the retrieved content
        3. If the retrieved content doesn't answer the question, say so
        4. Maintain a helpful and educational tone
        5. Provide accurate information based on the book content
        """
        
    async def retrieve_book_content(self, query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Mock retrieval function that simulates the behavior without API calls
        """
        try:
            # Use the retrieval pipeline to get relevant chunks
            # For mock purposes, we'll create some sample results
            mock_results = [
                {
                    "content": f"Sample content related to '{query}' from the Physical AI & Humanoid Robotics course.",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/intro",
                    "title": "Introduction to Physical AI",
                    "score": 0.9
                },
                {
                    "content": f"More information about {query} in the context of robotics.",
                    "url": "https://hackathon1-book-ragchatbot.vercel.app/docs/modules/ros2/intro",
                    "title": "ROS 2 Introduction",
                    "score": 0.8
                }
            ]
            
            return {
                "query": query,
                "results": mock_results,
                "total_results": len(mock_results)
            }
        except Exception as e:
            logger.error(f"Error in mock retrieve_book_content: {str(e)}")
            return {"error": f"Retrieval failed: {str(e)}"}
    
    async def chat(self, user_message: str) -> Dict[str, Any]:
        """
        Mock chat function that simulates agent response without API calls
        """
        try:
            # Simulate retrieval
            retrieval_result = await self.retrieve_book_content(user_message)
            
            if "error" in retrieval_result:
                return {
                    "response": "I'm sorry, but I encountered an error while processing your request.",
                    "success": False,
                    "error": retrieval_result["error"]
                }
            
            # Generate a mock response based on the retrieved content
            mock_response = f"Based on the book content, here's what I found about '{user_message}':\n\n"
            for result in retrieval_result["results"]:
                mock_response += f"- {result['content']}\n"
                mock_response += f"  Source: {result['title']} - {result['url']}\n\n"
            
            mock_response += "This is a simulated response using the OpenAI Agents SDK structure without actual API calls."
            
            return {
                "response": mock_response,
                "success": True
            }
        except Exception as e:
            logger.error(f"Error in mock chat: {str(e)}")
            return {
                "response": "I'm sorry, but I encountered an error while processing your request.",
                "success": False,
                "error": str(e)
            }
    
    async def validate_response_grounding(self, query: str, response: str) -> Dict[str, Any]:
        """
        Validate that the response is grounded in the book content
        """
        # Retrieve content for the same query
        query_embedding = await self.retrieval_pipeline.generate_query_embedding(query)
        if not query_embedding:
            return {"error": "Failed to validate response grounding"}
        
        retrieved_chunks = await self.retrieval_pipeline.search_similar_chunks(query_embedding, top_k=5)
        
        # Simple validation: check if response contains content from retrieved chunks
        contains_retrieved_content = False
        relevant_sources = []
        
        for chunk in retrieved_chunks:
            if len(chunk["content"]) > 20:  # Only check substantial content
                if chunk["content"].lower() in response.lower():
                    contains_retrieved_content = True
                    relevant_sources.append({
                        "url": chunk["url"],
                        "title": chunk["title"],
                        "content_preview": chunk["content"][:100] + "..."
                    })
        
        return {
            "query": query,
            "grounded_in_retrieved_content": contains_retrieved_content,
            "relevant_sources_count": len(relevant_sources),
            "relevant_sources": relevant_sources,
            "validation_performed": True
        }


class MockRAGAgentTester:
    """
    Class to test the Mock RAG Agent functionality
    """
    
    def __init__(self, rag_agent: MockRAGAgent):
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
        Run comprehensive test of the mock RAG agent
        """
        logger.info("Starting mock RAG agent test...")
        
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
        report.append("MOCK RAG AGENT TEST REPORT")
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
    Main function to demonstrate the Mock RAG Agent
    """
    logger.info("Initializing Mock RAG Agent with OpenAI Agents SDK structure...")
    
    # Initialize the Mock RAG Agent
    rag_agent = MockRAGAgent()
    
    # Create tester
    tester = MockRAGAgentTester(rag_agent)
    
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