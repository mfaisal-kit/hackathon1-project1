import React from 'react';
import Layout from '@theme/Layout';
import Chatbot from '../components/Chatbot/Chatbot';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function ChatPage() {
  const { siteConfig } = useDocusaurusContext();
  
  return (
    <Layout
      title={`Chat - ${siteConfig.title}`}
      description="AI Chatbot for Physical AI & Humanoid Robotics Course">
      <main>
        <div style={{ maxWidth: '800px', margin: '0 auto', padding: '2rem 1rem' }}>
          <h1>Book AI Assistant</h1>
          <p>
            Ask questions about the Physical AI & Humanoid Robotics course content. 
            Our AI assistant will provide answers based on the book material.
          </p>
          <div style={{ marginTop: '2rem' }}>
            <Chatbot />
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default ChatPage;