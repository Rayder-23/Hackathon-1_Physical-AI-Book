import React from 'react';
import { usePersonalization } from '@site/src/contexts/PersonalizationContext';

// Personalized section that can be filtered by background or difficulty
const PersonalizedSection = ({
  children,
  forBackground,
  forDifficulty,
  className = '',
  style = {},
  ...props
}) => {
  const { settings, isInitialized } = usePersonalization();

  // If context is not initialized, show section normally
  if (!isInitialized) {
    return <div className={className} style={style} {...props}>{children}</div>;
  }

  // If personalization is disabled, show section normally
  if (!settings.enabled) {
    return <div className={className} style={style} {...props}>{children}</div>;
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

  return <div className={className} style={style} {...props}>{children}</div>;
};

export default PersonalizedSection;