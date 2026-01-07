"""
Script to test the API endpoint functionality
"""
import os
import asyncio
import requests
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def test_api_endpoint():
    # Test the query endpoint directly
    api_url = "http://127.0.0.1:8001/query"  # Assuming your backend is running locally on port 8001
    
    # Sample query
    test_query = {
        "query": "What is ROS 2?",
        "top_k": 3
    }
    
    try:
        response = requests.post(api_url, json=test_query)
        print(f"Status Code: {response.status_code}")
        
        if response.status_code == 200:
            result = response.json()
            print(f"Response: {result}")
        else:
            print(f"Error: {response.text}")
    except Exception as e:
        print(f"Could not connect to API: {str(e)}")
        print("Make sure your backend API is running on http://127.0.0.1:8001")

def test_health_check():
    # Test the health endpoint
    api_url = "http://127.0.0.1:8001/health"
    
    try:
        response = requests.get(api_url)
        print(f"Health Check Status Code: {response.status_code}")
        
        if response.status_code == 200:
            result = response.json()
            print(f"Health Check Response: {result}")
        else:
            print(f"Health Check Error: {response.text}")
    except Exception as e:
        print(f"Could not connect to API for health check: {str(e)}")

if __name__ == "__main__":
    print("Testing API endpoints...")
    test_health_check()
    print("\nTesting query endpoint...")
    test_api_endpoint()