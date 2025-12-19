// Index file to export all personalized MDX components
import PersonalizedContent from './PersonalizedContent';
import PersonalizedParagraph from './PersonalizedParagraph';
import PersonalizedCode from './PersonalizedCode';
import PersonalizedSection from './PersonalizedSection';
import TranslatableContent from './TranslatableContent';
import ProtectedContent from './ProtectedContent';

// Default export for Docusaurus MDX components
const MDXComponents = {
  PersonalizedContent,
  PersonalizedParagraph,
  PersonalizedCode,
  PersonalizedSection,
  TranslatableContent,
  ProtectedContent,
};

export default MDXComponents;

// Named exports for individual components
export {
  PersonalizedContent,
  PersonalizedParagraph,
  PersonalizedCode,
  PersonalizedSection,
  TranslatableContent,
  ProtectedContent
};