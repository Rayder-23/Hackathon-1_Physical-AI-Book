// Security Context for Physical AI Book
// Manages security-related state and provides security utilities throughout the app

import React, { createContext, useContext, useReducer } from 'react';
import {
  sanitizeInput,
  validateEmail,
  validatePassword,
  sanitizeUserProfile,
  checkContentAccess,
  validateContentForStorage,
  generateSecureToken,
  RateLimiter,
  initializeSecurity
} from '../utils/security';

// Define the initial state for security
const initialState = {
  securityLevel: 'standard', // 'basic', 'standard', 'enhanced'
  rateLimiter: new RateLimiter(60000, 10), // 10 requests per minute
  initialized: false,
  securityAudit: null,
  suspiciousActivity: [],
  contentFilter: {
    enabled: true,
    allowedDomains: ['localhost', 'your-domain.com', 'humanoid-robotics-book.pages.dev'],
    blockedPatterns: []
  }
};

// Define action types
const actionTypes = {
  SET_SECURITY_LEVEL: 'SET_SECURITY_LEVEL',
  ADD_SUSPICIOUS_ACTIVITY: 'ADD_SUSPICIOUS_ACTIVITY',
  CLEAR_SUSPICIOUS_ACTIVITY: 'CLEAR_SUSPICIOUS_ACTIVITY',
  RUN_SECURITY_AUDIT: 'RUN_SECURITY_AUDIT',
  UPDATE_CONTENT_FILTER: 'UPDATE_CONTENT_FILTER',
  INITIALIZE_SECURITY: 'INITIALIZE_SECURITY'
};

// Define the reducer
const securityReducer = (state, action) => {
  switch (action.type) {
    case actionTypes.SET_SECURITY_LEVEL:
      return {
        ...state,
        securityLevel: action.payload
      };

    case actionTypes.ADD_SUSPICIOUS_ACTIVITY:
      return {
        ...state,
        suspiciousActivity: [...state.suspiciousActivity, {
          ...action.payload,
          timestamp: new Date().toISOString(),
          id: Date.now()
        }]
      };

    case actionTypes.CLEAR_SUSPICIOUS_ACTIVITY:
      return {
        ...state,
        suspiciousActivity: []
      };

    case actionTypes.RUN_SECURITY_AUDIT:
      return {
        ...state,
        securityAudit: action.payload
      };

    case actionTypes.UPDATE_CONTENT_FILTER:
      return {
        ...state,
        contentFilter: {
          ...state.contentFilter,
          ...action.payload
        }
      };

    case actionTypes.INITIALIZE_SECURITY:
      return {
        ...state,
        initialized: true
      };

    default:
      return state;
  }
};

// Create the context
const SecurityContext = createContext();

// Create the provider component
export const SecurityProvider = ({ children }) => {
  const [state, dispatch] = useReducer(securityReducer, initialState);

  // Initialize security measures when component mounts
  React.useEffect(() => {
    initializeSecurity();

    // Run initial security audit
    const auditResults = {
      localStorage: true, // Placeholder - would call checkLocalStorageSecurity()
      sessionStorage: true, // Placeholder - would call checkSessionStorageSecurity()
      dom: true, // Placeholder - would call checkDOMSecurity()
      inputs: true, // Placeholder - would call checkInputSecurity()
      cookies: true, // Placeholder - would call checkCookieSecurity()
      timestamp: new Date().toISOString()
    };

    dispatch({ type: actionTypes.RUN_SECURITY_AUDIT, payload: auditResults });
    dispatch({ type: actionTypes.INITIALIZE_SECURITY });
  }, []);

  // Security functions
  const setSecurityLevel = (level) => {
    dispatch({ type: actionTypes.SET_SECURITY_LEVEL, payload: level });
  };

  const addSuspiciousActivity = (activity) => {
    dispatch({ type: actionTypes.ADD_SUSPICIOUS_ACTIVITY, payload: activity });
  };

  const clearSuspiciousActivity = () => {
    dispatch({ type: actionTypes.CLEAR_SUSPICIOUS_ACTIVITY });
  };

  const runSecurityAudit = () => {
    // In a real implementation, this would run the actual security audit
    // For now, we'll return a mock result
    const auditResults = {
      localStorage: true,
      sessionStorage: true,
      dom: true,
      inputs: true,
      cookies: true,
      timestamp: new Date().toISOString()
    };

    dispatch({ type: actionTypes.RUN_SECURITY_AUDIT, payload: auditResults });
    return auditResults;
  };

  const updateContentFilter = (filterSettings) => {
    dispatch({ type: actionTypes.UPDATE_CONTENT_FILTER, payload: filterSettings });
  };

  const checkRateLimit = (identifier) => {
    return state.rateLimiter.isAllowed(identifier);
  };

  const validateUserInput = (input, inputType = 'generic') => {
    // Sanitize the input
    const sanitizedInput = sanitizeInput(input);

    // Perform type-specific validation
    switch (inputType) {
      case 'email':
        return {
          isValid: validateEmail(sanitizedInput),
          sanitizedValue: sanitizedInput
        };

      case 'password':
        return {
          isValid: validatePassword(sanitizedInput),
          sanitizedValue: sanitizedInput
        };

      case 'profile':
        return {
          isValid: true, // Profile validation would be more complex
          sanitizedValue: sanitizeUserProfile(sanitizedInput)
        };

      default:
        return {
          isValid: sanitizedInput.length > 0,
          sanitizedValue: sanitizedInput
        };
    }
  };

  const validateContentAccess = (user, contentMetadata) => {
    return checkContentAccess(user, contentMetadata);
  };

  const validateContentForStorageSecurely = (content, contentType) => {
    return validateContentForStorage(content, contentType);
  };

  const generateCSRFToken = () => {
    return generateSecureToken();
  };

  const value = {
    ...state,
    setSecurityLevel,
    addSuspiciousActivity,
    clearSuspiciousActivity,
    runSecurityAudit,
    updateContentFilter,
    checkRateLimit: checkRateLimit,
    validateUserInput,
    validateContentAccess,
    validateContentForStorageSecurely,
    generateCSRFToken: generateCSRFToken
  };

  return (
    <SecurityContext.Provider value={value}>
      {children}
    </SecurityContext.Provider>
  );
};

// Custom hook to use the security context
export const useSecurity = () => {
  const context = useContext(SecurityContext);
  if (!context) {
    // In SSR or if not wrapped properly, return a default state
    if (typeof window === 'undefined') {
      return {
        securityLevel: 'standard',
        rateLimiter: new RateLimiter(60000, 10),
        initialized: false,
        securityAudit: null,
        suspiciousActivity: [],
        contentFilter: {
          enabled: true,
          allowedDomains: ['localhost', 'your-domain.com', 'humanoid-robotics-book.pages.dev'],
          blockedPatterns: []
        },
        setSecurityLevel: () => {},
        addSuspiciousActivity: () => {},
        clearSuspiciousActivity: () => {},
        runSecurityAudit: () => {},
        updateContentFilter: () => {},
        checkRateLimit: () => true,
        validateUserInput: (input) => ({ isValid: true, sanitizedValue: input }),
        validateContentAccess: () => true,
        validateContentForStorageSecurely: (content) => ({ valid: true, content }),
        generateCSRFToken: () => 'mock-token'
      };
    } else {
      throw new Error('useSecurity must be used within a SecurityProvider');
    }
  }
  return context;
};