// User profile storage utilities for the Physical AI Book
// Handles user profile data storage and retrieval in localStorage

// Get user profile from localStorage
export const getUserProfile = () => {
  try {
    const profile = localStorage.getItem('userProfile');
    return profile ? JSON.parse(profile) : null;
  } catch (error) {
    console.error("Error getting user profile:", error);
    return null;
  }
};

// Save user profile to localStorage
export const saveUserProfile = (profileData) => {
  try {
    localStorage.setItem('userProfile', JSON.stringify(profileData));
    return true;
  } catch (error) {
    console.error("Error saving user profile:", error);
    return false;
  }
};

// Update specific fields in user profile
export const updateUserProfileFields = (fieldsToUpdate) => {
  try {
    const existingProfile = getUserProfile() || {};
    const updatedProfile = {
      ...existingProfile,
      ...fieldsToUpdate,
      updatedAt: new Date().toISOString()
    };

    return saveUserProfile(updatedProfile);
  } catch (error) {
    console.error("Error updating user profile fields:", error);
    return false;
  }
};

// Get user's software and hardware background
export const getUserBackground = () => {
  try {
    const profile = getUserProfile();
    if (!profile) {
      return {
        softwareBackground: '',
        hardwareBackground: '',
        experienceLevel: 'intermediate'
      };
    }

    return {
      softwareBackground: profile.softwareBackground || '',
      hardwareBackground: profile.hardwareBackground || '',
      experienceLevel: profile.experienceLevel || 'intermediate'
    };
  } catch (error) {
    console.error("Error getting user background:", error);
    return {
      softwareBackground: '',
      hardwareBackground: '',
      experienceLevel: 'intermediate'
    };
  }
};

// Set user's software and hardware background
export const setUserBackground = (softwareBackground, hardwareBackground) => {
  return updateUserProfileFields({
    softwareBackground,
    hardwareBackground
  });
};

// Check if user has completed the questionnaire
export const hasCompletedQuestionnaire = () => {
  try {
    const profile = getUserProfile();
    return !!(profile && profile.completedQuestionnaire);
  } catch (error) {
    console.error("Error checking questionnaire completion:", error);
    return false;
  }
};

// Mark questionnaire as completed
export const markQuestionnaireComplete = () => {
  return updateUserProfileFields({
    completedQuestionnaire: true,
    questionnaireCompletedAt: new Date().toISOString()
  });
};

// Get user's personalization preferences
export const getUserPreferences = () => {
  try {
    const profile = getUserProfile();
    return profile?.preferences || {};
  } catch (error) {
    console.error("Error getting user preferences:", error);
    return {};
  }
};

// Set user's personalization preferences
export const setUserPreferences = (preferences) => {
  return updateUserProfileFields({
    preferences: {
      ...getUserPreferences(),
      ...preferences
    }
  });
};

// Clear user profile (for logout or reset)
export const clearUserProfile = () => {
  try {
    localStorage.removeItem('userProfile');
    return true;
  } catch (error) {
    console.error("Error clearing user profile:", error);
    return false;
  }
};

// Initialize user profile if it doesn't exist
export const initializeUserProfile = (initialData = {}) => {
  const existingProfile = getUserProfile();
  if (!existingProfile) {
    const defaultProfile = {
      id: generateUserId(),
      createdAt: new Date().toISOString(),
      ...initialData
    };
    return saveUserProfile(defaultProfile);
  }
  return true;
};

// Generate a simple user ID
const generateUserId = () => {
  return 'user_' + Date.now().toString(36) + Math.random().toString(36).substr(2, 5);
};