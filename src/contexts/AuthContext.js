import React, { createContext, useContext, useReducer } from 'react';
import { signIn, signOut, getSession, getUserProfile } from '../config/auth';

// Define the initial state for authentication
const initialState = {
  user: null,
  isAuthenticated: false,
  loading: true,
  error: null
};

// Define action types
const actionTypes = {
  LOGIN_START: 'LOGIN_START',
  LOGIN_SUCCESS: 'LOGIN_SUCCESS',
  LOGIN_FAILURE: 'LOGIN_FAILURE',
  LOGOUT: 'LOGOUT',
  CHECK_SESSION_START: 'CHECK_SESSION_START',
  CHECK_SESSION_SUCCESS: 'CHECK_SESSION_SUCCESS',
  CHECK_SESSION_FAILURE: 'CHECK_SESSION_FAILURE'
};

// Define the reducer
const authReducer = (state, action) => {
  switch (action.type) {
    case actionTypes.LOGIN_START:
      return {
        ...state,
        loading: true,
        error: null
      };

    case actionTypes.LOGIN_SUCCESS:
      return {
        ...state,
        user: action.payload.user,
        isAuthenticated: true,
        loading: false,
        error: null
      };

    case actionTypes.LOGIN_FAILURE:
      return {
        ...state,
        user: null,
        isAuthenticated: false,
        loading: false,
        error: action.payload.error
      };

    case actionTypes.LOGOUT:
      return {
        ...state,
        user: null,
        isAuthenticated: false,
        loading: false,
        error: null
      };

    case actionTypes.CHECK_SESSION_START:
      return {
        ...state,
        loading: true
      };

    case actionTypes.CHECK_SESSION_SUCCESS:
      return {
        ...state,
        user: action.payload.user,
        isAuthenticated: !!action.payload.user,
        loading: false
      };

    case actionTypes.CHECK_SESSION_FAILURE:
      return {
        ...state,
        user: null,
        isAuthenticated: false,
        loading: false
      };

    default:
      return state;
  }
};

// Create the context
const AuthContext = createContext();

// Create the provider component
export const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);

  // Check session on initial load
  React.useEffect(() => {
    const checkSession = async () => {
      dispatch({ type: actionTypes.CHECK_SESSION_START });
      try {
        // In static mode, we'll check localStorage for user data
        const user = await getUserProfile();
        if (user) {
          dispatch({
            type: actionTypes.CHECK_SESSION_SUCCESS,
            payload: { user }
          });
        } else {
          dispatch({ type: actionTypes.CHECK_SESSION_SUCCESS, payload: { user: null } });
        }
      } catch (error) {
        dispatch({ type: actionTypes.CHECK_SESSION_FAILURE });
      }
    };

    checkSession();
  }, []);

  // Login function
  const login = async (email, password) => {
    dispatch({ type: actionTypes.LOGIN_START });

    try {
      // For static hosting, we'll simulate login by storing user data in localStorage
      // In a real implementation, this would make an API call to a backend
      const user = {
        id: `user_${Date.now()}`,
        email,
        name: email.split('@')[0], // Simple name from email
        role: 'user', // Default role
        createdAt: new Date().toISOString()
      };

      // Store user data in localStorage
      localStorage.setItem('currentUser', JSON.stringify(user));

      dispatch({
        type: actionTypes.LOGIN_SUCCESS,
        payload: { user }
      });

      return { success: true, user };
    } catch (error) {
      const errorMessage = error.message || 'Login failed';
      dispatch({
        type: actionTypes.LOGIN_FAILURE,
        payload: { error: errorMessage }
      });
      return { success: false, error: errorMessage };
    }
  };

  // Logout function
  const logout = async () => {
    try {
      // Clear user data from localStorage
      localStorage.removeItem('currentUser');
      localStorage.removeItem('userProfile');

      // Call the signOut function from better-auth
      await signOut();

      dispatch({ type: actionTypes.LOGOUT });
    } catch (error) {
      console.error('Logout error:', error);
      dispatch({ type: actionTypes.LOGOUT });
    }
  };

  const value = {
    ...state,
    login,
    logout
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use the auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};