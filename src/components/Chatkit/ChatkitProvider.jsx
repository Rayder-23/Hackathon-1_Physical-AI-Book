import React from 'react';

// The new @openai/chatkit-react library doesn't require a provider pattern
// The useChatKit hook handles the provider functionality internally
// This component is kept for compatibility with the existing architecture
// but is essentially a passthrough component

export const ChatkitProvider = ({ children }) => {
  return children;
};

// For backward compatibility with existing code that uses useChatkit
export const useChatkit = () => {
  // This hook is no longer needed since the new library handles state internally
  // Return a simple object to maintain compatibility
  return { isConnected: true };
};

export default ChatkitProvider;