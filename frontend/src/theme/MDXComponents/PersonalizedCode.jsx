import React from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';

// Personalized code block that can be filtered by background or difficulty
const PersonalizedCode = ({
  children,
  forBackground,
  forDifficulty,
  className = '',
  ...props
}) => {
  // Check if we're in browser environment before using context
  if (typeof window === 'undefined') {
    // In SSR, show code normally to avoid context errors
    return <code className={className} {...props}>{children}</code>;
  }

  const { settings, isInitialized } = usePersonalization();

  // If context is not initialized, show code normally
  if (!isInitialized) {
    return <code className={className} {...props}>{children}</code>;
  }

  // If personalization is disabled, show code normally
  if (!settings.enabled) {
    return <code className={className} {...props}>{children}</code>;
  }

  // Apply filters based on settings
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

      if (userLevel && requiredLevel && userLevel < requiredLevel) {
        return null;
      }
    }
  }

  return <code className={className} {...props}>{children}</code>;
};

export default PersonalizedCode;