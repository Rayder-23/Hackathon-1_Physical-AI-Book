import React, { createContext, useContext, useState, useEffect } from 'react';
import { useAuth } from './AuthContext';

const PersonalizationContext = createContext();

export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
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
      if (user) {
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
      // Mark as initialized after loading settings
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