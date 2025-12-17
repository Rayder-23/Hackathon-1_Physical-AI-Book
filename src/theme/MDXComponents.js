import React from 'react';
import OriginalMDXComponents from '@theme-original/MDXComponents';
import PersonalizedContent from './MDXComponents/PersonalizedContent';
import PersonalizedParagraph from './MDXComponents/PersonalizedParagraph';
import PersonalizedCode from './MDXComponents/PersonalizedCode';
import PersonalizedSection from './MDXComponents/PersonalizedSection';
import TranslatableContent from './MDXComponents/TranslatableContent';
import ProtectedContent from './MDXComponents/ProtectedContent';

// Custom MDX components that include personalization, translation, and authentication
const MDXComponents = {
  // Spread the original components
  ...OriginalMDXComponents,

  // Add our custom personalized content components
  PersonalizedContent,
  PersonalizedParagraph,
  PersonalizedCode,
  PersonalizedSection,
  TranslatableContent,
  ProtectedContent,

  // You can add more personalized components here as needed
  // For example, personalized headings, etc.
};

// Named exports for individual components
export {
  PersonalizedContent,
  PersonalizedParagraph,
  PersonalizedCode,
  PersonalizedSection,
  TranslatableContent,
  ProtectedContent
};

export default MDXComponents;