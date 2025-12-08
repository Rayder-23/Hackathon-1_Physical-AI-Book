import React, { createContext, useContext, useReducer } from 'react';
import { getTranslationSettings, saveTranslationSettings } from '../utils/translation';

// Define the initial state for translation
const initialState = {
  currentLanguage: getTranslationSettings().language || 'en',
  isInitialized: false
};

// Define action types
const actionTypes = {
  SET_LANGUAGE: 'SET_LANGUAGE',
  INITIALIZE: 'INITIALIZE'
};

// Define the reducer
const translationReducer = (state, action) => {
  switch (action.type) {
    case actionTypes.SET_LANGUAGE:
      const newSettings = {
        ...getTranslationSettings(),
        language: action.payload
      };
      saveTranslationSettings(newSettings);
      return {
        ...state,
        currentLanguage: action.payload
      };

    case actionTypes.INITIALIZE:
      return {
        ...state,
        isInitialized: true
      };

    default:
      return state;
  }
};

// Create the context
const TranslationContext = createContext();

// Create the provider component
export const TranslationProvider = ({ children }) => {
  const [state, dispatch] = useReducer(translationReducer, initialState);

  // Initialize the context
  React.useEffect(() => {
    if (!state.isInitialized) {
      dispatch({ type: actionTypes.INITIALIZE });
    }
  }, [state.isInitialized]);

  // Actions
  const switchLanguage = (language) => {
    dispatch({ type: actionTypes.SET_LANGUAGE, payload: language });
  };

  const value = {
    ...state,
    switchLanguage
  };

  return (
    <TranslationContext.Provider value={value}>
      {children}
    </TranslationContext.Provider>
  );
};

// Custom hook to use the translation context
export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) {
    // In SSR or if not wrapped properly, return a default state
    if (typeof window === 'undefined') {
      return {
        currentLanguage: 'en',
        isInitialized: true, // Default to initialized for SSR
        switchLanguage: () => {},
        toggleLanguage: () => {}
      };
    } else {
      throw new Error('useTranslation must be used within a TranslationProvider');
    }
  }
  return context;
};

// Export action types for use elsewhere if needed
export { actionTypes };