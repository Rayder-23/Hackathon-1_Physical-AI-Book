import React from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';

// Personalized paragraph that can be filtered by background or difficulty
const PersonalizedParagraph = ({
  children,
  forBackground,
  forDifficulty,
  className = '',
  ...props
}) => {
  // Check if we're in browser environment before using context
  if (typeof window === 'undefined') {
    // In SSR, show content normally to avoid context errors
    return <p className={className} {...props}>{children}</p>;
  }

  const { settings, isInitialized } = usePersonalization();

  // If context is not initialized, show content normally
  if (!isInitialized) {
    return <p className={className} {...props}>{children}</p>;
  }

  // If personalization is disabled, show content normally
  if (!settings.enabled) {
    return <p className={className} {...props}>{children}</p>;
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

  return <p className={className} {...props}>{children}</p>;
};

export default PersonalizedParagraph;