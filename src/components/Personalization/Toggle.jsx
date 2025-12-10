import React, { useState, useEffect } from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';

const PersonalizationToggle = () => {
  // Check if we're in browser environment before using context
  if (typeof window === 'undefined') {
    // In SSR, return a placeholder to avoid context errors
    // The actual component will be rendered on the client side
    return <div>Loading personalization settings...</div>;
  }

  const {
    settings,
    togglePersonalization,
    setDifficulty,
    setBackgroundPreference,
    isInitialized
  } = usePersonalization();

  const [localEnabled, setLocalEnabled] = useState(settings?.enabled || false);
  const [localDifficulty, setLocalDifficulty] = useState(settings?.difficulty || 'intermediate');
  const [localBackground, setLocalBackground] = useState(settings?.background || 'mixed');

  // Update local state when context settings change
  useEffect(() => {
    if (settings) {
      setLocalEnabled(settings.enabled);
      setLocalDifficulty(settings.difficulty);
      setLocalBackground(settings.background);
    }
  }, [settings]);

  const handleToggleChange = () => {
    togglePersonalization();
  };

  const handleDifficultyChange = (e) => {
    const newDifficulty = e.target.value;
    setLocalDifficulty(newDifficulty);
    setDifficulty(newDifficulty);
  };

  const handleBackgroundChange = (e) => {
    const newBackground = e.target.value;
    setLocalBackground(newBackground);
    setBackgroundPreference(newBackground);
  };

  // Don't render if context is not initialized yet
  if (!isInitialized) {
    return <div>Loading personalization settings...</div>;
  }

  return (
    <div className="personalization-toggle">
      <div className="row">
        <div className="col col--6">
          <label className="checkbox-container">
            <input
              type="checkbox"
              checked={localEnabled}
              onChange={handleToggleChange}
            />
            <span className="checkbox-label">Enable Personalization</span>
          </label>
        </div>

        {localEnabled && (
          <>
            <div className="col col--3">
              <label htmlFor="difficulty-select">
                <small>Difficulty Level:</small>
              </label>
              <select
                id="difficulty-select"
                value={localDifficulty}
                onChange={handleDifficultyChange}
                className="dropdown dropdown--sm margin-left--sm"
              >
                <option value="beginner">Beginner</option>
                <option value="intermediate">Intermediate</option>
                <option value="advanced">Advanced</option>
              </select>
            </div>

            <div className="col col--3">
              <label htmlFor="background-select">
                <small>Focus:</small>
              </label>
              <select
                id="background-select"
                value={localBackground}
                onChange={handleBackgroundChange}
                className="dropdown dropdown--sm margin-left--sm"
              >
                <option value="mixed">Mixed</option>
                <option value="software">Software</option>
                <option value="hardware">Hardware</option>
              </select>
            </div>
          </>
        )}
      </div>

      {localEnabled && (
        <div className="margin-top--sm">
          <small className="text--muted">
            Personalization is enabled. Content will be adapted based on your preferences.
          </small>
        </div>
      )}
    </div>
  );
};

export default PersonalizationToggle;
export { PersonalizationToggle };