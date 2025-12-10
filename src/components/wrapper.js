// Client module to wrap the app with all required providers
import React from 'react';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import { TranslationProvider } from '../contexts/TranslationContext';
import { AuthProvider } from '../contexts/AuthContext';
import { SecurityProvider } from '../contexts/SecurityContext';

// Check if we're in a browser environment before applying the wrapper
export const wrapRootElement = ({ element }) => {
  // In SSR (server-side rendering), we return the element as-is
  // The providers will be initialized on the client side
  if (typeof window === 'undefined') {
    return element;
  }

  // In browser environment, wrap with providers
  return (
    <SecurityProvider>
      <AuthProvider>
        <TranslationProvider>
          <PersonalizationProvider>
            {element}
          </PersonalizationProvider>
        </TranslationProvider>
      </AuthProvider>
    </SecurityProvider>
  );
};