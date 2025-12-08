// Client module to wrap the app with both Personalization and Translation providers
import React from 'react';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import { TranslationProvider } from '../contexts/TranslationContext';

export const wrapRootElement = ({ element }) => {
  return (
    <TranslationProvider>
      <PersonalizationProvider>
        {element}
      </PersonalizationProvider>
    </TranslationProvider>
  );
};