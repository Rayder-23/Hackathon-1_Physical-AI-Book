import React, { useState, useEffect } from 'react';
import { useTranslation } from '../../contexts/TranslationContext';

const TranslationToggle = () => {
  // Check if we're in browser environment before using context
  if (typeof window === 'undefined') {
    // In SSR, return a placeholder to avoid context errors
    // The actual component will be rendered on the client side
    return <div>Loading translation settings...</div>;
  }

  const { currentLanguage, switchLanguage, isInitialized } = useTranslation();
  const [localLanguage, setLocalLanguage] = useState(currentLanguage || 'en');

  // Update local state when context language changes
  useEffect(() => {
    if (currentLanguage) {
      setLocalLanguage(currentLanguage);
    }
  }, [currentLanguage]);

  const handleLanguageChange = (e) => {
    const newLanguage = e.target.value;
    setLocalLanguage(newLanguage);
    switchLanguage(newLanguage);
  };

  // Don't render if context is not initialized yet
  if (!isInitialized) {
    return <div>Loading translation settings...</div>;
  }

  return (
    <div className="translation-toggle">
      <div className="row">
        <div className="col col--6">
          <label htmlFor="language-select">
            <small>Language:</small>
          </label>
          <select
            id="language-select"
            value={localLanguage}
            onChange={handleLanguageChange}
            className="dropdown dropdown--sm margin-left--sm"
          >
            <option value="en">English</option>
            <option value="ur">Urdu</option>
          </select>
        </div>
      </div>

      <div className="margin-top--sm">
        <small className="text--muted">
          {localLanguage === 'ur'
            ? 'متن اردو میں دکھایا جا رہا ہے'
            : 'Content is displayed in English'}
        </small>
      </div>
    </div>
  );
};

export default TranslationToggle;
export { TranslationToggle };