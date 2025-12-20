import React, { createContext, useContext, useState, useEffect } from 'react';

// Create Auth Context
const AuthContext = createContext();

// Auth Provider Component
export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [profile, setProfile] = useState(null);
  const [loading, setLoading] = useState(true);
  const [token, setToken] = useState(null);

  // Check authentication status on initial load
  useEffect(() => {
    const checkAuthStatus = async () => {
      try {
        // Check if we're in browser environment
        if (typeof window !== 'undefined') {
          // Try to get user from localStorage
          const storedUser = localStorage.getItem('currentUser');
          if (storedUser) {
            const userData = JSON.parse(storedUser);
            setUser(userData);
          }

          // Try to get profile from localStorage
          const storedProfile = localStorage.getItem('userProfile');
          if (storedProfile) {
            const profileData = JSON.parse(storedProfile);
            setProfile(profileData);
          }

          // Try to get token from localStorage
          const storedToken = localStorage.getItem('authToken');
          if (storedToken) {
            setToken(storedToken);
          }
        }
      } catch (error) {
        console.error('Error checking auth status:', error);
      } finally {
        setLoading(false);
      }
    };

    checkAuthStatus();
  }, []);

  // Register function with profile data
  const register = async (email, password, softwareExperience, hardwareExperience, backgroundPreference) => {
    try {
      setLoading(true);

      // Use environment variable or default for backend URL
      const backendUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
        ? process.env.REACT_APP_BACKEND_URL
        : 'http://localhost:8005';
      // Make API call to backend
      const response = await fetch(`${backendUrl}/api/auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          software_experience: softwareExperience,
          hardware_experience: hardwareExperience,
          background_preference: backgroundPreference
        }),
      });

      // Check if the response is JSON before parsing
      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        // If not JSON, try to read as text to see what we got
        const text = await response.text();
        console.error('Non-JSON response from server:', text);
        throw new Error(`Server returned non-JSON response. Status: ${response.status}`);
      }

      const data = await response.json();

      if (data.success) {
        setUser(data.user);
        setProfile(data.profile);
        setToken(data.token);

        // Store user data in localStorage for static hosting
        if (typeof window !== 'undefined') {
          localStorage.setItem('currentUser', JSON.stringify(data.user));
          localStorage.setItem('userProfile', JSON.stringify(data.profile));
          localStorage.setItem('authToken', data.token);
        }

        return { success: true, user: data.user, profile: data.profile };
      } else {
        return { success: false, error: data.error };
      }
    } catch (error) {
      console.error('Registration error:', error);
      return { success: false, error: error.message };
    } finally {
      setLoading(false);
    }
  };

  // Login function
  const login = async (email, password) => {
    try {
      setLoading(true);

      // Use environment variable or default for backend URL
      const backendUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
        ? process.env.REACT_APP_BACKEND_URL
        : 'http://localhost:8005';
      // Make API call to backend
      const response = await fetch(`${backendUrl}/api/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password
        }),
      });

      // Check if the response is JSON before parsing
      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        // If not JSON, try to read as text to see what we got
        const text = await response.text();
        console.error('Non-JSON response from server:', text);
        throw new Error(`Server returned non-JSON response. Status: ${response.status}`);
      }

      const data = await response.json();

      if (data.success) {
        setUser(data.user);
        setProfile(data.profile);
        setToken(data.token);

        // Store user data in localStorage for static hosting
        if (typeof window !== 'undefined') {
          localStorage.setItem('currentUser', JSON.stringify(data.user));
          localStorage.setItem('userProfile', JSON.stringify(data.profile));
          localStorage.setItem('authToken', data.token);
        }

        return { success: true, user: data.user, profile: data.profile };
      } else {
        return { success: false, error: data.error };
      }
    } catch (error) {
      console.error('Login error:', error);
      return { success: false, error: error.message };
    } finally {
      setLoading(false);
    }
  };

  // Logout function
  const logout = async () => {
    try {
      setLoading(true);

      // Call backend logout if token exists
      if (token) {
        // Use environment variable or default for backend URL
        const backendUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
          ? process.env.REACT_APP_BACKEND_URL
          : 'http://localhost:8005';
        await fetch(`${backendUrl}/api/auth/logout`, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        });
      }

      // Clear local state
      setUser(null);
      setProfile(null);
      setToken(null);

      // Clear localStorage
      if (typeof window !== 'undefined') {
        localStorage.removeItem('currentUser');
        localStorage.removeItem('userProfile');
        localStorage.removeItem('authToken');
      }
    } catch (error) {
      console.error('Logout error:', error);
    } finally {
      setLoading(false);
    }
  };

  // Update profile function
  const updateProfile = async (profileData) => {
    try {
      if (!token) {
        throw new Error('Not authenticated');
      }

      // Use environment variable or default for backend URL
      const backendUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
        ? process.env.REACT_APP_BACKEND_URL
        : 'http://localhost:8005';
      const response = await fetch(`${backendUrl}/api/auth/profile`, {
        method: 'PUT',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(profileData),
      });

      // Check if the response is JSON before parsing
      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        // If not JSON, try to read as text to see what we got
        const text = await response.text();
        console.error('Non-JSON response from server:', text);
        throw new Error(`Server returned non-JSON response. Status: ${response.status}`);
      }

      const data = await response.json();

      if (data.success) {
        setProfile(data.profile);

        // Update profile in localStorage as well
        if (typeof window !== 'undefined') {
          localStorage.setItem('userProfile', JSON.stringify(data.profile));
        }

        return { success: true, profile: data.profile };
      } else {
        return { success: false, error: data.error };
      }
    } catch (error) {
      console.error('Update profile error:', error);
      return { success: false, error: error.message };
    }
  };

  // Get profile function
  const getProfile = async () => {
    try {
      if (!token) {
        throw new Error('Not authenticated');
      }

      // Use environment variable or default for backend URL
      const backendUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
        ? process.env.REACT_APP_BACKEND_URL
        : 'http://localhost:8005';
      const response = await fetch(`${backendUrl}/api/auth/profile`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
      });

      // Check if the response is JSON before parsing
      const contentType = response.headers.get('content-type');
      if (!contentType || !contentType.includes('application/json')) {
        // If not JSON, try to read as text to see what we got
        const text = await response.text();
        console.error('Non-JSON response from server:', text);
        throw new Error(`Server returned non-JSON response. Status: ${response.status}`);
      }

      const data = await response.json();

      if (data.success) {
        setProfile(data.profile);

        // Update profile in localStorage as well
        if (typeof window !== 'undefined') {
          localStorage.setItem('userProfile', JSON.stringify(data.profile));
        }

        return { success: true, profile: data.profile };
      } else {
        return { success: false, error: data.error };
      }
    } catch (error) {
      console.error('Get profile error:', error);
      return { success: false, error: error.message };
    }
  };

  // Context value
  const value = {
    user,
    profile,
    loading,
    token,
    register,
    login,
    logout,
    updateProfile,
    getProfile,
    isAuthenticated: !!user
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use Auth Context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};