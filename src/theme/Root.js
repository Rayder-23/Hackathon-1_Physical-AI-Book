// File: src/theme/Root.js
import React from 'react';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import { TranslationProvider } from '../contexts/TranslationContext';
import { AuthProvider } from '../contexts/AuthContext';
import { SecurityProvider } from '../contexts/SecurityContext';

export default function Root({ children }) {
  // During SSR, do not rely on browser APIs
  const isBrowser = typeof window !== 'undefined';

  if (!isBrowser) {
    // Return children unwrapped during SSR
    return <>{children}</>;
  }

  return (
    <SecurityProvider>
      <AuthProvider>
        <TranslationProvider>
          <PersonalizationProvider>
            {children}
          </PersonalizationProvider>
        </TranslationProvider>
      </AuthProvider>
    </SecurityProvider>
  );
}