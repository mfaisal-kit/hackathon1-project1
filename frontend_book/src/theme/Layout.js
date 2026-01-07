import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import FloatingChatWidget from '../components/Chatbot/FloatingChatWidget';

function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
      </OriginalLayout>
      <FloatingChatWidget />
    </>
  );
}

export default Layout;