# Personalization Flow Test

This document outlines how to test the personalization flow in the Physical AI Book.

## Test Steps

1. Navigate to the Personalization Test page in the sidebar under "Testing" → "Personalization Test Page"
2. Complete the "Tell Us About Your Background" questionnaire
3. Verify that your profile information is saved in localStorage
4. Use the "Enable Personalization" toggle to turn on/off personalization
5. Adjust difficulty level and background preferences using the dropdowns
6. Verify that content appears/disappears based on your preferences:
   - Content marked for "software" background should only show when background is set to "Software"
   - Content marked for "hardware" background should only show when background is set to "Hardware"
   - Content marked for specific difficulty levels should only show when your level meets or exceeds the requirement
7. Test that when personalization is disabled, all content appears regardless of filters
8. Test that when personalization is enabled but filters don't match, appropriate content is hidden

## Expected Results

- User profile data should be stored in localStorage and persist between page reloads
- The personalization toggle should immediately affect content visibility
- Difficulty and background preferences should filter content appropriately
- Personalized components (PersonalizedContent, PersonalizedParagraph, PersonalizedSection) should respond to context changes
- The signup questionnaire should properly save user background information