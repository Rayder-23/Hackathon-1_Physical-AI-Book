import React from 'react';
import { useTranslation } from '@site/src/contexts/TranslationContext';
import { hasTranslation, createFallbackContent } from '@site/src/utils/translation';

// Component to handle content that can be translated
const TranslatableContent = ({
  children,
  urduContent,
  englishContent,
  contentId, // Unique identifier for this content piece
  fallbackToEnglish = true,
  className = ''
}) => {
  const { currentLanguage, isInitialized } = useTranslation();

  // If context is not initialized, show default content
  if (!isInitialized) {
    return <div className={className}>{children || englishContent}</div>;
  }

  // Determine which content to show based on language
  let contentToShow = children || englishContent;

  if (currentLanguage === 'ur') {
    if (urduContent) {
      // If we have explicit Urdu content, use it
      contentToShow = urduContent;
    } else if (contentId) {
      // Check if translation exists using our mapping system
      if (hasTranslation(contentId, 'ur')) {
        // Translation exists, but we don't have the content here
        // This would typically happen if content is loaded dynamically
        contentToShow = children || englishContent;
      } else {
        // No translation available, show fallback
        if (fallbackToEnglish) {
          const fallbackContent = createFallbackContent(
            children || englishContent,
            'en',
            'ur'
          );
          contentToShow = <div dangerouslySetInnerHTML={{ __html: fallbackContent }} />;
        }
      }
    } else if (fallbackToEnglish) {
      // No contentId provided, create fallback directly
      const fallbackContent = createFallbackContent(
        children || englishContent,
        'en',
        'ur'
      );
      contentToShow = <div dangerouslySetInnerHTML={{ __html: fallbackContent }} />;
    }
  }

  return <div className={className}>{contentToShow}</div>;
};

export default TranslatableContent;