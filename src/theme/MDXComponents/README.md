# Personalized MDX Components

This directory contains custom MDX components that provide personalization features for the Physical AI Book.

## Components

### PersonalizedContent
A wrapper component that shows/hides content based on user preferences.

Props:
- `forBackground`: 'software', 'hardware', or 'mixed' - Only show content if user's background matches
- `forDifficulty`: 'beginner', 'intermediate', 'advanced' - Only show content if user's difficulty level matches or exceeds
- `showIfEnabled`: boolean - If true, show when personalization is enabled; if false, show when disabled

### PersonalizedParagraph
A paragraph element that can be filtered by background or difficulty.

Props:
- `forBackground`: 'software', 'hardware', or 'mixed'
- `forDifficulty`: 'beginner', 'intermediate', 'advanced'
- Other standard paragraph props

### PersonalizedCode
A code element that can be filtered by background or difficulty.

Props:
- `forBackground`: 'software', 'hardware', or 'mixed'
- `forDifficulty`: 'beginner', 'intermediate', 'advanced'
- Other standard code props

### PersonalizedSection
A div element that can be filtered by background or difficulty.

Props:
- `forBackground`: 'software', 'hardware', or 'mixed'
- `forDifficulty`: 'beginner', 'intermediate', 'advanced'
- Other standard div props

## Usage

To use these components in MDX files:

```mdx
import { PersonalizedContent } from '@site/src/theme/MDXComponents';

<PersonalizedContent forBackground="software" forDifficulty="advanced">
  This content will only show to users with a software background and advanced level.
</PersonalizedContent>
```