import React from 'react';
import Layout from '@theme/Layout';
import FloatingChatWidget from '../components/Chatbot/FloatingChatWidget';

function Root({children}) {
  return (
    <>
      <Layout>
        {children}
      </Layout>
      <FloatingChatWidget />
    </>
  );
}

export default Root;