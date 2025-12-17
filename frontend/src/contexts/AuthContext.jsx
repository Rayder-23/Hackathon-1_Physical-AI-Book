import React, { createContext, useContext, useState, useEffect } from 'react';
import authClient from '../utils/authClient';

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    // During SSR or if not wrapped in provider, return a default context
    if (typeof window === 'undefined') {
      // Server-side rendering - return default values
      return {
        user: null,
        login: () => Promise.resolve({ success: false, error: 'Auth not available server-side' }),
        logout: () => Promise.resolve(),
        register: () => Promise.resolve({ success: false, error: 'Auth not available server-side' }),
        loading: false,
        isAuthenticated: false
      };
    } else {
      // Client-side without provider - throw error as before
      throw new Error('useAuth must be used within an AuthProvider');
    }
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  // Initialize auth state from Better-Auth
  useEffect(() => {
    if (typeof window !== 'undefined') {
      // Get current session from Better-Auth
      const checkSession = async () => {
        try {
          const session = await authClient.getSession();
          if (session?.user) {
            setUser(session.user);
            setIsAuthenticated(true);
          }
        } catch (e) {
          console.error('Failed to get session', e);
        } finally {
          setLoading(false);
        }
      };

      checkSession();
    } else {
      // On server, set loading to false immediately
      setLoading(false);
    }
  }, []);

  const login = async (email, password) => {
    if (typeof window !== 'undefined') {
      try {
        // Use Better-Auth client login
        const result = await authClient.signIn.email({
          email,
          password,
          callbackURL: window.location.origin + '/Hackathon-1_Physical-AI-Book/' // Redirect after login with base URL
        });

        if (result?.user) {
          setUser(result.user);
          setIsAuthenticated(true);
          return { success: true, user: result.user };
        } else {
          return { success: false, error: result?.error?.message || 'Login failed' };
        }
      } catch (error) {
        console.error('Login error:', error);
        return { success: false, error: error.message || 'Login failed' };
      }
    }
    return { success: false, error: 'Window not available' };
  };

  const logout = async () => {
    if (typeof window !== 'undefined') {
      try {
        // Use Better-Auth client logout
        await authClient.signOut();
        setUser(null);
        setIsAuthenticated(false);
      } catch (error) {
        console.error('Logout error:', error);
        // Still clear local state even if API call fails
        setUser(null);
        setIsAuthenticated(false);
      }
    }
  };

  const register = async (email, password, name) => {
    if (typeof window !== 'undefined') {
      try {
        // Use Better-Auth client registration
        // Make email optional by using a placeholder if not provided
        const emailToUse = email || `user-${Date.now()}@example.com`;
        const result = await authClient.signUp.email({
          email: emailToUse,
          password,
          name,
          callbackURL: window.location.origin + '/Hackathon-1_Physical-AI-Book/' // Redirect after registration with base URL
        });

        if (result?.user) {
          setUser(result.user);
          setIsAuthenticated(true);
          return { success: true, user: result.user };
        } else {
          return { success: false, error: result?.error?.message || 'Registration failed' };
        }
      } catch (error) {
        console.error('Registration error:', error);
        return { success: false, error: error.message || 'Registration failed' };
      }
    }
    return { success: false, error: 'Window not available' };
  };

  const value = {
    user,
    login,
    logout,
    register,
    loading,
    isAuthenticated
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export default AuthContext;