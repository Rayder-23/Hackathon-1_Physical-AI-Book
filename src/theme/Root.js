import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import { TranslationProvider } from '../contexts/TranslationContext';
import ChatInterface from '../components/Chatkit/ChatInterface';

// Root component that wraps the entire app with context providers
// All providers are rendered on both server and client to maintain consistent DOM structure
// The providers themselves handle SSR internally by checking for window availability
export default function Root({ children }) {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        <TranslationProvider>
          <>
            {children}
            <ChatInterface />
          </>
        </TranslationProvider>
      </PersonalizationProvider>
    </AuthProvider>
  );
}