// Client module to wrap the app with all required providers
import React from 'react';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import { TranslationProvider } from '../contexts/TranslationContext';
import { AuthProvider } from '../contexts/AuthContext';

export const wrapRootElement = ({ element }) => {
  return (
    <AuthProvider>
      <TranslationProvider>
        <PersonalizationProvider>
          {element}
        </PersonalizationProvider>
      </TranslationProvider>
    </AuthProvider>
  );
};