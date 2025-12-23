import React from 'react';
import FloatingChatbot from './components/FloatingChatbot';

// This component will be rendered once on every page
const Root = ({ children }) => {
  return (
    <>
      {children}
      <FloatingChatbot />
    </>
  );
};

export default Root;