import { useContext } from 'react';
import AuthContext from '../contexts/AuthContext';

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