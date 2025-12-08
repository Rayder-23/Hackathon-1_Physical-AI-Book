// Translation utility functions for the Physical AI Book
// Handles language switching between English and Urdu with fallback support

// Get current language from localStorage or default to English
export const getCurrentLanguage = () => {
  try {
    const lang = localStorage.getItem('preferredLanguage');
    return lang || 'en'; // Default to English
  } catch (error) {
    console.error("Error getting current language:", error);
    return 'en';
  }
};

// Set current language and save to localStorage
export const setCurrentLanguage = (languageCode) => {
  try {
    localStorage.setItem('preferredLanguage', languageCode);
    // Update HTML lang attribute for accessibility
    document.documentElement.lang = languageCode;
    return true;
  } catch (error) {
    console.error("Error setting current language:", error);
    return false;
  }
};

// Switch language and handle page reload if needed
export const switchLanguage = (languageCode) => {
  const currentLang = getCurrentLanguage();
  if (currentLang === languageCode) {
    return; // Already on this language
  }

  // Set the new language
  setCurrentLanguage(languageCode);

  // Optionally reload the page to apply the new language
  // In a real implementation, you might want to update content dynamically
  // without a full page reload
  window.location.reload();
};

// Get the Urdu equivalent path for an English path
export const getUrduPath = (englishPath) => {
  // Convert English path to Urdu path
  // e.g., /docs/intro -> /urdu/intro
  if (englishPath.startsWith('/docs/')) {
    return englishPath.replace('/docs/', '/urdu/');
  }
  // For paths that don't start with /docs/, handle them differently
  return `/urdu${englishPath}`;
};

// Check if Urdu translation exists for a given English path
export const hasUrduTranslation = (englishPath) => {
  // In a static site, we'd need to check if the Urdu file exists
  // This is a simplified implementation - in practice, you'd check file existence
  try {
    // Check if the Urdu version exists in static files
    // This would typically be handled by the build system
    const urduPath = getUrduPath(englishPath);
    // For now, we'll assume translation status is stored in a mapping
    const translationMap = JSON.parse(localStorage.getItem('translationMap') || '{}');
    return !!translationMap[urduPath];
  } catch (error) {
    console.error("Error checking Urdu translation:", error);
    return false;
  }
};

// Get fallback content when Urdu translation is not available
export const getFallbackContent = (englishContent) => {
  // Return English content with a notice that it's in English
  // because Urdu translation is not available
  const fallbackNotice = `
> [!NOTE]
> This content is currently only available in English.
> A ${getCurrentLanguage() === 'ur' ? 'Urdu' : 'Urdu'} translation is not yet available.
>
> `;

  return fallbackNotice + englishContent;
};

// Load content in the preferred language with fallback
export const loadContentWithFallback = async (englishPath, options = {}) => {
  const {
    fallbackToEnglish = true,
    includeFallbackNotice = true
  } = options;

  const currentLang = getCurrentLanguage();

  if (currentLang === 'ur' && hasUrduTranslation(englishPath)) {
    // Try to load Urdu content
    try {
      // In a real implementation, this would fetch from the Urdu path
      const urduPath = getUrduPath(englishPath);
      // For now, we'll return a placeholder indicating Urdu content would be loaded
      return `Urdu translation for: ${urduPath}`;
    } catch (error) {
      console.warn(`Urdu translation not available for ${englishPath}, falling back to English`);
    }
  }

  // Load English content (or fallback)
  try {
    // In a real implementation, this would fetch from the English path
    // For now, return a placeholder
    if (includeFallbackNotice && currentLang === 'ur') {
      return getFallbackContent(`English content for: ${englishPath}`);
    }
    return `English content for: ${englishPath}`;
  } catch (error) {
    console.error("Error loading content:", error);
    return "Error loading content";
  }
};

// Initialize translation system
export const initializeTranslationSystem = () => {
  // Set up event listeners for language switching
  // This would typically be called when the page loads
  const currentLang = getCurrentLanguage();
  document.documentElement.lang = currentLang;

  // Add a class to the body for language-specific styling
  document.body.classList.add(`lang-${currentLang}`);

  return currentLang;
};

// Get language direction (ltr for English, rtl for Urdu)
export const getLanguageDirection = (languageCode = null) => {
  const lang = languageCode || getCurrentLanguage();
  // Urdu is written right-to-left
  return lang === 'ur' ? 'rtl' : 'ltr';
};

// Update document direction based on current language
export const updateDocumentDirection = () => {
  const direction = getLanguageDirection();
  document.documentElement.dir = direction;
  document.body.setAttribute('dir', direction);
};

// Get translation settings from localStorage
export const getTranslationSettings = () => {
  try {
    const settings = localStorage.getItem('translationSettings');
    return settings ? JSON.parse(settings) : { language: 'en' };
  } catch (error) {
    console.error("Error getting translation settings:", error);
    return { language: 'en' };
  }
};

// Save translation settings to localStorage
export const saveTranslationSettings = (settings) => {
  try {
    localStorage.setItem('translationSettings', JSON.stringify(settings));
    // Update HTML lang attribute for accessibility
    document.documentElement.lang = settings.language;
    // Update document direction
    updateDocumentDirection();
    return true;
  } catch (error) {
    console.error("Error saving translation settings:", error);
    return false;
  }
};

// Check if translation exists for a given content ID
export const hasTranslation = (contentId, language = null) => {
  const targetLang = language || getCurrentLanguage();
  if (targetLang === 'en') return true; // English is always available

  try {
    // Check if translation mapping exists in localStorage
    const translationMap = JSON.parse(localStorage.getItem('translationMap') || '{}');
    const langMap = translationMap[targetLang] || {};
    return !!langMap[contentId] && langMap[contentId].available;
  } catch (error) {
    console.error("Error checking translation availability:", error);
    return false;
  }
};

// Create fallback content when translation is not available
export const createFallbackContent = (originalContent, originalLanguage = 'en', targetLanguage = 'ur') => {
  // Create a fallback message based on the target language
  let fallbackMessage = '';

  if (targetLanguage === 'ur') {
    fallbackMessage = `
> [!NOTE]
> یہ مواد اردو میں دستیاب نہیں ہے۔
> This content is not available in Urdu.
>
> `;
  } else {
    fallbackMessage = `
> [!NOTE]
> This content is not available in the selected language.
> یہ مواد منتخب کردہ زبان میں دستیاب نہیں ہے۔
>
> `;
  }

  return fallbackMessage + originalContent;
};

// Update translation mapping to track available translations
export const updateTranslationMap = (contentId, language, available = true) => {
  try {
    const translationMap = JSON.parse(localStorage.getItem('translationMap') || '{}');
    if (!translationMap[language]) {
      translationMap[language] = {};
    }
    translationMap[language][contentId] = { available, lastUpdated: new Date().toISOString() };
    localStorage.setItem('translationMap', JSON.stringify(translationMap));
    return true;
  } catch (error) {
    console.error("Error updating translation map:", error);
    return false;
  }
};