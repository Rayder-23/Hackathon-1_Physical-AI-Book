import React from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';

// Component to show/hide content based on personalization settings
const PersonalizedContent = ({
  children,
  forBackground,  // 'software', 'hardware', or 'mixed'
  forDifficulty,  // 'beginner', 'intermediate', 'advanced'
  showIfEnabled = true // If true, show when personalization is enabled; if false, show when disabled
}) => {
  // Check if we're in browser environment before using context
  if (typeof window === 'undefined') {
    // In SSR, show all content to avoid context errors
    return <>{children}</>;
  }

  const { settings, isInitialized } = usePersonalization();

  // If context is not initialized, show all content
  if (!isInitialized) {
    return <>{children}</>;
  }

  // If personalization is disabled, show content based on showIfEnabled
  if (!settings.enabled) {
    return showIfEnabled ? null : <>{children}</>;
  }

  // If personalization is enabled, check filters
  if (settings.enabled) {
    // Check background filter
    if (forBackground && settings.background && settings.background !== 'mixed') {
      if (forBackground !== settings.background) {
        return null;
      }
    }

    // Check difficulty filter
    if (forDifficulty && settings.difficulty) {
      const difficultyLevels = {
        'beginner': 1,
        'intermediate': 2,
        'advanced': 3
      };

      const requiredLevel = difficultyLevels[forDifficulty];
      const userLevel = difficultyLevels[settings.difficulty];

      // Only show content if user's level is equal or greater than required level
      if (userLevel && requiredLevel && userLevel < requiredLevel) {
        return null;
      }
    }
  }

  return <>{children}</>;
};

export default PersonalizedContent;