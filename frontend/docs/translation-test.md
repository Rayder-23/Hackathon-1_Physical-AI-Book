---
title: Translation Test Page
---

# Translation Test Page

This page demonstrates the Urdu translation features of the Physical AI Book.

## Translation Toggle

import { TranslationToggle } from '../src/components/Translation/Toggle';

<TranslationToggle />

## Translatable Content Examples

import { TranslatableContent } from '../src/theme/MDXComponents';

### Example with Content ID

<TranslatableContent contentId="example-1">
This is sample English content that would have a Urdu translation. When the language is switched to Urdu, this content will either show the Urdu translation or a fallback message if the translation is not available.
</TranslatableContent>

### Example without Content ID

<TranslatableContent>
This is another example of translatable content. Since no contentId is provided, it will show a fallback message when switched to Urdu.
</TranslatableContent>

### Example with Explicit Urdu Content

<TranslatableContent
  urduContent="یہ مثال کا اردو ورژن ہے۔ یہ صرف اردو میں دکھایا جائے گا۔"
  englishContent="This is the English version of the example. This will be shown in English."
  contentId="example-3"
>
Default content that will be overridden by the explicit English content above.
</TranslatableContent>