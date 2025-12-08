---
title: Personalization Test Page
---

# Personalization Test Page

This page demonstrates the personalization features of the Physical AI Book.

## Questionnaire

import { SignupQuestionnaire } from '@site/src/components/Auth/SignupQuestionnaire';

<SignupQuestionnaire onComplete={(data) => console.log('Questionnaire completed:', data)} />

## Personalization Toggle

import { PersonalizationToggle } from '@site/src/components/Personalization/Toggle';

<PersonalizationToggle />

## Personalized Content Examples

import { PersonalizedContent, PersonalizedParagraph, PersonalizedSection } from '@site/src/theme/MDXComponents';

### Content for Software Background

<PersonalizedContent forBackground="software">
  <p>This content is specifically for users with a software background. You'll see this if your background preference is set to "Software".</p>
</PersonalizedContent>

### Content for Hardware Background

<PersonalizedContent forBackground="hardware">
  <p>This content is specifically for users with a hardware background. You'll see this if your background preference is set to "Hardware".</p>
</PersonalizedContent>

### Advanced Difficulty Content

<PersonalizedContent forDifficulty="advanced">
  <p>This content is for advanced users only. You'll see this if your difficulty level is set to "Advanced".</p>
</PersonalizedContent>

### Beginner/Intermediate Content

<PersonalizedContent forDifficulty="beginner">
  <p>This content is for beginner users. You'll see this if your difficulty level is set to "Beginner".</p>
</PersonalizedContent>

<PersonalizedContent forDifficulty="intermediate">
  <p>This content is for intermediate users. You'll see this if your difficulty level is set to "Intermediate".</p>
</PersonalizedContent>

### Mixed Background Content

<PersonalizedContent forBackground="mixed">
  <p>This content is for users with mixed backgrounds or when background preference is set to "Mixed".</p>
</PersonalizedContent>

### Personalized Paragraph Example

<PersonalizedParagraph forBackground="software" forDifficulty="advanced">
  This is a personalized paragraph that will only show to users with a software background at an advanced level.
</PersonalizedParagraph>

### Personalized Section Example

<PersonalizedSection forBackground="hardware" className="alert alert--info">
  <h3>Hardware Focus Section</h3>
  <p>This section is specifically for hardware-focused learners.</p>
</PersonalizedSection>