import React, { createContext, useContext, useState, useEffect } from 'react';
import { useAuth } from '../hooks/useAuth';

const PersonalizationContext = createContext();

export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (!context) {
    // During SSR or if not wrapped in provider, return a default context
    if (typeof window === 'undefined') {
      // Server-side rendering - return default values
      return {
        settings: {
          enabled: false,
          difficulty: 'intermediate',
          background: 'mixed'
        },
        updateSettings: () => {},
        isInitialized: true, // Consider initialized on server for default values
        togglePersonalization: () => {},
        setDifficulty: () => {},
        setBackgroundPreference: () => {}
      };
    } else {
      // Client-side without provider - throw error as before
      throw new Error('usePersonalization must be used within a PersonalizationProvider');
    }
  }
  return context;
};

export const PersonalizationProvider = ({ children }) => {
  const { user } = useAuth();
  const [isInitialized, setIsInitialized] = useState(false);
  const [settings, setSettings] = useState({
    enabled: false,
    difficulty: 'intermediate',
    background: 'mixed'
  });

  // Load saved settings from localStorage when user is available
  useEffect(() => {
    if (typeof window !== 'undefined') {
      if (user?.id) { // Only proceed if user exists and has an ID
        const savedSettings = localStorage.getItem(`personalization-${user.id}`);
        if (savedSettings) {
          try {
            const parsedSettings = JSON.parse(savedSettings);
            setSettings(parsedSettings);
          } catch (e) {
            console.error('Failed to parse personalization settings', e);
          }
        }
      }
      // Mark as initialized after attempting to load settings
      setIsInitialized(true);
    } else {
      // On server, mark as initialized immediately
      setIsInitialized(true);
    }
  }, [user]);

  const updateSettings = (newSettings) => {
    if (typeof window !== 'undefined') {
      setSettings(prev => {
        const updated = { ...prev, ...newSettings };
        // Store in localStorage for persistence if user is available
        if (user) {
          localStorage.setItem(`personalization-${user.id}`, JSON.stringify(updated));
        }
        return updated;
      });
    }
  };

  const togglePersonalization = () => {
    updateSettings({ enabled: !settings.enabled });
  };

  const setDifficulty = (difficulty) => {
    updateSettings({ difficulty });
  };

  const setBackgroundPreference = (background) => {
    updateSettings({ background });
  };

  return (
    <PersonalizationContext.Provider value={{
      settings,
      updateSettings,
      isInitialized,
      togglePersonalization,
      setDifficulty,
      setBackgroundPreference
    }}>
      {children}
    </PersonalizationContext.Provider>
  );
};

export default PersonalizationContext;