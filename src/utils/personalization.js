// Personalization utility functions for the Physical AI Book
// Handles user preferences and content adaptation based on user profile

// Get current personalization settings from localStorage
export const getPersonalizationSettings = () => {
  try {
    const settings = localStorage.getItem('personalizationSettings');
    return settings ? JSON.parse(settings) : {
      enabled: false,
      difficulty: 'intermediate', // 'beginner', 'intermediate', 'advanced'
      background: 'mixed', // 'software', 'hardware', 'mixed'
      preferences: {}
    };
  } catch (error) {
    console.error("Error getting personalization settings:", error);
    return {
      enabled: false,
      difficulty: 'intermediate',
      background: 'mixed',
      preferences: {}
    };
  }
};

// Save personalization settings to localStorage
export const savePersonalizationSettings = (settings) => {
  try {
    localStorage.setItem('personalizationSettings', JSON.stringify(settings));
    return true;
  } catch (error) {
    console.error("Error saving personalization settings:", error);
    return false;
  }
};

// Toggle personalization on/off
export const togglePersonalization = () => {
  const currentSettings = getPersonalizationSettings();
  const newSettings = {
    ...currentSettings,
    enabled: !currentSettings.enabled
  };
  return savePersonalizationSettings(newSettings);
};

// Set difficulty level
export const setDifficultyLevel = (level) => {
  const currentSettings = getPersonalizationSettings();
  const newSettings = {
    ...currentSettings,
    difficulty: level
  };
  return savePersonalizationSettings(newSettings);
};

// Set user background preference
export const setUserBackground = (background) => {
  const currentSettings = getPersonalizationSettings();
  const newSettings = {
    ...currentSettings,
    background: background
  };
  return savePersonalizationSettings(newSettings);
};

// Apply personalization to content based on user profile
export const personalizeContent = (content, userProfile = null) => {
  const settings = getPersonalizationSettings();

  if (!settings.enabled) {
    return content;
  }

  // If user profile is provided, use it for more specific personalization
  const userBackground = userProfile || getUserBackgroundFromStorage();

  // Apply personalization based on difficulty level and user background
  let personalizedContent = content;

  // Adjust content based on difficulty level
  if (settings.difficulty === 'beginner') {
    // Simplify complex concepts, add more explanations
    personalizedContent = simplifyContent(personalizedContent);
  } else if (settings.difficulty === 'advanced') {
    // Add more technical depth, advanced examples
    personalizedContent = enhanceContent(personalizedContent);
  }

  // Adjust content based on user background
  if (userBackground.softwareBackground && userBackground.hardwareBackground) {
    // Mixed background - balance software and hardware content
    personalizedContent = balanceContent(personalizedContent);
  } else if (userBackground.softwareBackground) {
    // Software-focused background - emphasize software aspects
    personalizedContent = emphasizeSoftware(personalizedContent);
  } else if (userBackground.hardwareBackground) {
    // Hardware-focused background - emphasize hardware aspects
    personalizedContent = emphasizeHardware(personalizedContent);
  }

  return personalizedContent;
};

// Helper functions for content personalization
const simplifyContent = (content) => {
  // Replace complex technical terms with simpler explanations
  // This is a placeholder implementation - actual implementation would be more sophisticated
  return content.replace(/(advanced|complex|sophisticated)/g, 'basic');
};

const enhanceContent = (content) => {
  // Add more technical depth and advanced examples
  // This is a placeholder implementation - actual implementation would be more sophisticated
  return content.replace(/(basic|simple)/g, 'advanced');
};

const balanceContent = (content) => {
  // Balance software and hardware content
  // This is a placeholder implementation
  return content;
};

const emphasizeSoftware = (content) => {
  // Emphasize software aspects of the content
  // This is a placeholder implementation
  return content;
};

const emphasizeHardware = (content) => {
  // Emphasize hardware aspects of the content
  // This is a placeholder implementation
  return content;
};

const getUserBackgroundFromStorage = () => {
  try {
    const profile = localStorage.getItem('userProfile');
    return profile ? JSON.parse(profile) : {};
  } catch (error) {
    console.error("Error getting user background:", error);
    return {};
  }
};

// Get content variant based on personalization settings
export const getContentVariant = (defaultContent, personalizedContent) => {
  const settings = getPersonalizationSettings();

  if (settings.enabled && personalizedContent) {
    return personalizeContent(personalizedContent);
  }

  return defaultContent;
};