// frontend_book/src/components/Chatbot/ApiService.js
// API service for frontend-backend communication

class ApiService {
  constructor() {
    // Use environment variable if available, otherwise default to Hugging Face Space
    // Note: Hugging Face Spaces expose the API on the root URL, not on a specific port
    this.baseUrl = typeof process !== 'undefined' && process.env?.REACT_APP_API_URL
      ? process.env.REACT_APP_API_URL
      : 'https://muhammadfasial-deploy-ragchatbot.hf.space';
  }

  async query(requestData) {
    try {
      const response = await fetch(`${this.baseUrl}/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestData),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('API call failed:', error);
      // Return a mock response when backend is not available
      return {
        response: `I'm currently unable to connect to the backend service. This is a mock response. Please ensure the backend API is running on ${this.baseUrl}.`,
        session_id: 'mock-session',
        sources: [],
        success: false
      };
    }
  }

  async healthCheck() {
    try {
      const response = await fetch(`${this.baseUrl}/health`);
      return response.ok;
    } catch (error) {
      console.error('Health check failed:', error);
      return false;
    }
  }
}

export default new ApiService();