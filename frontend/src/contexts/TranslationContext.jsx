import React, { createContext, useContext, useState, useEffect } from 'react';

const TranslationContext = createContext();

export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) {
    // During SSR or if not wrapped in provider, return a default context
    if (typeof window === 'undefined') {
      // Server-side rendering - return default values
      return {
        currentLanguage: 'en',
        switchLanguage: () => {},
        isInitialized: true,
        translations: {},
        getTranslation: (key, defaultText) => defaultText,
        loading: false
      };
    } else {
      // Client-side without provider - throw error as before
      throw new Error('useTranslation must be used within a TranslationProvider');
    }
  }
  return context;
};

export const TranslationProvider = ({ children }) => {
  const [language, setLanguage] = useState('en');
  const [translations, setTranslations] = useState({});
  const [loading, setLoading] = useState(false);
  const [isInitialized, setIsInitialized] = useState(false);

  // Load Urdu translations when language is set to Urdu
  useEffect(() => {
    if (typeof window !== 'undefined') {
      if (language === 'ur') {
        setLoading(true);
        // Dynamic import for Urdu translations
        import('../data/ur/book.json')
          .then(module => {
            setTranslations(module.default || {});
            setLoading(false);
            setIsInitialized(true);
          })
          .catch(error => {
            console.error('Failed to load Urdu translations', error);
            setTranslations({});
            setLoading(false);
            setIsInitialized(true);
          });
      } else if (language === 'en') {
        setTranslations({});
        setLoading(false);
        setIsInitialized(true);
      }
    } else {
      // For SSR, mark as initialized immediately
      setIsInitialized(true);
    }
  }, [language]);

  const toggleLanguage = () => {
    setLanguage(prev => prev === 'en' ? 'ur' : 'en');
  };

  const switchLanguage = (newLanguage) => {
    setLanguage(newLanguage);
  };

  const getTranslation = (key, defaultText) => {
    if (language === 'ur' && translations[key]) {
      return translations[key];
    }
    return defaultText;
  };

  return (
    <TranslationContext.Provider value={{
      currentLanguage: language,
      switchLanguage,
      isInitialized,
      translations,
      getTranslation,
      loading
    }}>
      {children}
    </TranslationContext.Provider>
  );
};

export default TranslationContext;