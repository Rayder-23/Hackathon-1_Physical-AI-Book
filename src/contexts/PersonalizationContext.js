import React, { createContext, useContext, useReducer } from 'react';
import { getPersonalizationSettings, savePersonalizationSettings } from '../utils/personalization';
import { getUserBackground } from '../utils/userProfile';

// Define the initial state for personalization
const initialState = {
  settings: getPersonalizationSettings(),
  userBackground: getUserBackground(),
  isInitialized: false
};

// Define action types
const actionTypes = {
  UPDATE_SETTINGS: 'UPDATE_SETTINGS',
  UPDATE_USER_BACKGROUND: 'UPDATE_USER_BACKGROUND',
  TOGGLE_PERSONALIZATION: 'TOGGLE_PERSONALIZATION',
  SET_DIFFICULTY: 'SET_DIFFICULTY',
  SET_BACKGROUND_PREFERENCE: 'SET_BACKGROUND_PREFERENCE',
  INITIALIZE: 'INITIALIZE'
};

// Define the reducer
const personalizationReducer = (state, action) => {
  switch (action.type) {
    case actionTypes.UPDATE_SETTINGS:
      savePersonalizationSettings(action.payload);
      return {
        ...state,
        settings: action.payload
      };

    case actionTypes.UPDATE_USER_BACKGROUND:
      return {
        ...state,
        userBackground: action.payload
      };

    case actionTypes.TOGGLE_PERSONALIZATION:
      const newSettings = {
        ...state.settings,
        enabled: !state.settings.enabled
      };
      savePersonalizationSettings(newSettings);
      return {
        ...state,
        settings: newSettings
      };

    case actionTypes.SET_DIFFICULTY:
      const updatedSettings1 = {
        ...state.settings,
        difficulty: action.payload
      };
      savePersonalizationSettings(updatedSettings1);
      return {
        ...state,
        settings: updatedSettings1
      };

    case actionTypes.SET_BACKGROUND_PREFERENCE:
      const updatedSettings2 = {
        ...state.settings,
        background: action.payload
      };
      savePersonalizationSettings(updatedSettings2);
      return {
        ...state,
        settings: updatedSettings2
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
const PersonalizationContext = createContext();

// Create the provider component
export const PersonalizationProvider = ({ children }) => {
  const [state, dispatch] = useReducer(personalizationReducer, initialState);

  // Initialize the context
  React.useEffect(() => {
    if (!state.isInitialized) {
      dispatch({ type: actionTypes.INITIALIZE });
    }
  }, [state.isInitialized]);

  // Actions
  const updateSettings = (settings) => {
    dispatch({ type: actionTypes.UPDATE_SETTINGS, payload: settings });
  };

  const updateUserBackground = (background) => {
    dispatch({ type: actionTypes.UPDATE_USER_BACKGROUND, payload: background });
  };

  const togglePersonalization = () => {
    dispatch({ type: actionTypes.TOGGLE_PERSONALIZATION });
  };

  const setDifficulty = (level) => {
    dispatch({ type: actionTypes.SET_DIFFICULTY, payload: level });
  };

  const setBackgroundPreference = (background) => {
    dispatch({ type: actionTypes.SET_BACKGROUND_PREFERENCE, payload: background });
  };

  const value = {
    ...state,
    updateSettings,
    updateUserBackground,
    togglePersonalization,
    setDifficulty,
    setBackgroundPreference
  };

  return (
    <PersonalizationContext.Provider value={value}>
      {children}
    </PersonalizationContext.Provider>
  );
};

// Custom hook to use the personalization context
export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (!context) {
    // In SSR or if not wrapped properly, return a default state
    if (typeof window === 'undefined') {
      return {
        settings: { enabled: false, difficulty: 'intermediate', background: 'mixed' },
        userBackground: { softwareBackground: '', hardwareBackground: '', experienceLevel: 'intermediate' },
        isInitialized: true, // Default to initialized for SSR
        updateSettings: () => {},
        updateUserBackground: () => {},
        togglePersonalization: () => {},
        setDifficulty: () => {},
        setBackgroundPreference: () => {}
      };
    } else {
      throw new Error('usePersonalization must be used within a PersonalizationProvider');
    }
  }
  return context;
};

// Export action types for use elsewhere if needed
export { actionTypes };