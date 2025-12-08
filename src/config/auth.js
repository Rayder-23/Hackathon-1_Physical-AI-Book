// Better Auth configuration for static mode (GitHub Pages)
// This configuration works in static hosting environments

import { createAuthClient } from "better-auth/client";

// Client-side configuration for better-auth in static mode
const authConfig = {
  baseURL: typeof window !== 'undefined' ? window.location.origin : "http://localhost:3000",
  // For static hosting, we use the client-side only mode
  clientOptions: {
    // Define the pages that require authentication
    signInPage: "/auth/signin",
    signUpPage: "/auth/signup",
    verifyRequestPage: "/auth/verify-request",
  }
};

// Initialize the client
export const authClient = createAuthClient(authConfig);

// Export utility functions for authentication
export const signIn = authClient.signIn;
export const signOut = authClient.signOut;
export const signUp = authClient.signUp;
export const getSession = authClient.getSession;

// Enhanced user profile utility functions for static mode
export const getUserProfile = async () => {
  try {
    // First try to get session from better-auth
    const session = await getSession();
    if (session?.user) {
      return session.user;
    }

    // Fallback to localStorage for static hosting
    const storedUser = localStorage.getItem('currentUser');
    if (storedUser) {
      return JSON.parse(storedUser);
    }

    return null;
  } catch (error) {
    console.error("Error getting user profile:", error);
    // Fallback to localStorage in case of error
    try {
      const storedUser = localStorage.getItem('currentUser');
      return storedUser ? JSON.parse(storedUser) : null;
    } catch {
      return null;
    }
  }
};

export const updateUserProfile = async (profileData) => {
  // In static mode, we store user profile data in localStorage
  try {
    const existingProfile = JSON.parse(localStorage.getItem('userProfile') || '{}');
    const updatedProfile = {
      ...existingProfile,
      ...profileData,
      updatedAt: new Date().toISOString()
    };
    localStorage.setItem('userProfile', JSON.stringify(updatedProfile));
    return updatedProfile;
  } catch (error) {
    console.error("Error updating user profile:", error);
    return null;
  }
};

export const getUserBackground = () => {
  // Retrieve user's software/hardware background from localStorage
  try {
    const profile = JSON.parse(localStorage.getItem('userProfile') || '{}');
    return {
      softwareBackground: profile.softwareBackground || '',
      hardwareBackground: profile.hardwareBackground || '',
      experienceLevel: profile.experienceLevel || 'intermediate',
      completedQuestionnaire: profile.completedQuestionnaire || false
    };
  } catch (error) {
    console.error("Error getting user background:", error);
    return {
      softwareBackground: '',
      hardwareBackground: '',
      experienceLevel: 'intermediate',
      completedQuestionnaire: false
    };
  }
};

// Additional static mode authentication utilities
export const isAuthenticated = async () => {
  try {
    const user = await getUserProfile();
    return !!user;
  } catch (error) {
    console.error("Error checking authentication:", error);
    return false;
  }
};

export const getCurrentUser = () => {
  try {
    const storedUser = localStorage.getItem('currentUser');
    return storedUser ? JSON.parse(storedUser) : null;
  } catch (error) {
    console.error("Error getting current user:", error);
    return null;
  }
};

export const setCurrentUser = (userData) => {
  try {
    const userWithTimestamp = {
      ...userData,
      lastLogin: new Date().toISOString()
    };
    localStorage.setItem('currentUser', JSON.stringify(userWithTimestamp));
    return true;
  } catch (error) {
    console.error("Error setting current user:", error);
    return false;
  }
};

export const clearAuthData = () => {
  try {
    localStorage.removeItem('currentUser');
    localStorage.removeItem('userProfile');
    localStorage.removeItem('authToken'); // if using tokens
    return true;
  } catch (error) {
    console.error("Error clearing auth data:", error);
    return false;
  }
};