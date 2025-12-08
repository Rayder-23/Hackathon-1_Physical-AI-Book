# Translation Functionality Test

This document outlines how to test the Urdu translation features in the Physical AI Book.

## Test Steps

1. Navigate to the Translation Test page in the sidebar under "Testing" → "Translation Test Page"
2. Use the language toggle to switch between English and Urdu
3. Verify that the page language changes appropriately
4. Test the TranslatableContent component with different configurations:
   - Content with contentId but no Urdu translation (should show fallback)
   - Content with contentId and explicit Urdu content (should show Urdu when language is Urdu)
   - Content without contentId (should show fallback when language is Urdu)
5. Verify that the HTML lang attribute updates correctly
6. Check that the document direction (ltr/rtl) updates when switching to Urdu
7. Verify that localStorage is updated with the language preference
8. Test that the fallback mechanism shows appropriate notices when translations are not available

## Expected Results

- Language toggle should immediately change the displayed language
- Content should appear/disappear based on availability and language setting
- When Urdu is selected but translation is not available, fallback content with bilingual notice should be shown
- HTML attributes (lang, dir) should update to reflect the current language
- Language preference should persist between page reloads
- Translation mapping should be maintained in localStorage