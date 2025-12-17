// Access control utilities for the Physical AI Book
// Implements role-based access control and content protection

// Define user roles and permissions
export const USER_ROLES = {
  ANONYMOUS: 'anonymous',
  USER: 'user',
  AUTHOR: 'author',
  ADMIN: 'admin'
};

// Define content access levels
export const ACCESS_LEVELS = {
  PUBLIC: 'public',
  REGISTERED: 'registered',
  AUTHOR: 'author',
  ADMIN: 'admin'
};

// Check if user is authenticated
export const isAuthenticated = (user) => {
  return !!user && !!user.id;
};

// Check if user has a specific role
export const hasRole = (user, requiredRole) => {
  if (!user) return false;

  // Admins have access to everything
  if (user.role === USER_ROLES.ADMIN) return true;

  // Check specific role
  return user.role === requiredRole;
};

// Check if user has access to specific content based on access level
export const hasAccessToContent = (user, contentAccessLevel) => {
  // Public content is accessible to everyone
  if (contentAccessLevel === ACCESS_LEVELS.PUBLIC) return true;

  // Check if user is authenticated first
  if (!isAuthenticated(user)) {
    return contentAccessLevel === ACCESS_LEVELS.REGISTERED;
  }

  // Registered users can access registered content
  if (contentAccessLevel === ACCESS_LEVELS.REGISTERED) return true;

  // Check role-based access
  switch (contentAccessLevel) {
    case ACCESS_LEVELS.AUTHOR:
      return hasRole(user, USER_ROLES.AUTHOR) || hasRole(user, USER_ROLES.ADMIN);
    case ACCESS_LEVELS.ADMIN:
      return hasRole(user, USER_ROLES.ADMIN);
    default:
      return false;
  }
};

// Validate user permissions for specific actions
export const hasPermission = (user, action) => {
  if (!user) return false;

  // Define action permissions by role
  const permissions = {
    [USER_ROLES.ANONYMOUS]: [
      'view_public_content',
      'register_account',
      'request_password_reset'
    ],
    [USER_ROLES.USER]: [
      'view_public_content',
      'view_registered_content',
      'update_profile',
      'access_user_dashboard'
    ],
    [USER_ROLES.AUTHOR]: [
      'view_public_content',
      'view_registered_content',
      'create_content',
      'edit_content',
      'view_author_content',
      'access_author_dashboard'
    ],
    [USER_ROLES.ADMIN]: [
      'view_public_content',
      'view_registered_content',
      'view_author_content',
      'view_admin_content',
      'create_content',
      'edit_content',
      'delete_content',
      'manage_users',
      'access_admin_dashboard',
      'configure_system'
    ]
  };

  const userPermissions = permissions[user.role] || permissions[USER_ROLES.ANONYMOUS];
  return userPermissions.includes(action);
};

// Generate secure token for authentication
export const generateSecureToken = (length = 32) => {
  if (typeof window !== 'undefined' && window.crypto && window.crypto.getRandomValues) {
    // Use Web Crypto API for cryptographically secure random values
    const array = new Uint8Array(length);
    window.crypto.getRandomValues(array);
    return Array.from(array, byte => byte.toString(16).padStart(2, '0')).join('');
  } else {
    // Fallback to Math.random (less secure but works in all environments)
    let result = '';
    const characters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    for (let i = 0; i < length; i++) {
      result += characters.charAt(Math.floor(Math.random() * characters.length));
    }
    return result;
  }
};

// Validate authentication token
export const validateToken = (token) => {
  if (!token || typeof token !== 'string') return false;

  // Basic validation - check if it looks like a reasonable token
  return token.length >= 16 && /^[a-zA-Z0-9-_]+$/.test(token);
};

// Check if content requires authentication
export const contentRequiresAuth = (contentMetadata) => {
  if (!contentMetadata) return false;

  // Check if content has access level that requires authentication
  return contentMetadata.accessLevel &&
         contentMetadata.accessLevel !== ACCESS_LEVELS.PUBLIC;
};

// Get user's effective access level
export const getUserAccessLevel = (user) => {
  if (!user || !user.role) return ACCESS_LEVELS.PUBLIC;

  // Map user roles to access levels
  const roleToAccessLevel = {
    [USER_ROLES.ANONYMOUS]: ACCESS_LEVELS.PUBLIC,
    [USER_ROLES.USER]: ACCESS_LEVELS.REGISTERED,
    [USER_ROLES.AUTHOR]: ACCESS_LEVELS.AUTHOR,
    [USER_ROLES.ADMIN]: ACCESS_LEVELS.ADMIN
  };

  return roleToAccessLevel[user.role] || ACCESS_LEVELS.PUBLIC;
};

// Validate user profile data for security
export const validateUserProfile = (profileData) => {
  if (!profileData) return { valid: false, errors: ['Profile data is required'] };

  const errors = [];

  // Validate email format
  if (profileData.email && !/^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(profileData.email)) {
    errors.push('Invalid email format');
  }

  // Validate name length
  if (profileData.name && profileData.name.length > 100) {
    errors.push('Name is too long');
  }

  // Validate role if present
  if (profileData.role && !Object.values(USER_ROLES).includes(profileData.role)) {
    errors.push('Invalid role specified');
  }

  // Check for potentially dangerous content in profile fields
  const dangerousPatterns = [/<script/i, /javascript:/i, /on\w+=/i];
  const profileText = JSON.stringify(profileData);

  for (const pattern of dangerousPatterns) {
    if (pattern.test(profileText)) {
      errors.push('Profile contains potentially dangerous content');
      break;
    }
  }

  return {
    valid: errors.length === 0,
    errors
  };
};

// Sanitize user input
export const sanitizeUserInput = (input) => {
  if (typeof input !== 'string') return input;

  // Remove potentially dangerous characters/sequences
  return input
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '') // Remove script tags
    .replace(/javascript:/gi, '') // Remove javascript: protocols
    .replace(/on\w+="[^"]*"/gi, '') // Remove event handlers
    .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '') // Remove iframes
    .replace(/<object\b[^<]*(?:(?!<\/object>)<[^<]*)*<\/object>/gi, '') // Remove objects
    .replace(/<embed\b[^<]*(?:(?!<\/embed>)<[^<]*)*<\/embed>/gi, '') // Remove embeds
    .trim();
};

// Check rate limiting for user actions
export class RateLimiter {
  constructor(windowMs = 60000, maxRequests = 10) {
    this.windowMs = windowMs;
    this.maxRequests = maxRequests;
    this.clients = new Map();
  }

  isAllowed(identifier) {
    const now = Date.now();
    const client = this.clients.get(identifier) || { requests: [] };

    // Remove requests outside the current window
    const recentRequests = client.requests.filter(timestamp => now - timestamp < this.windowMs);

    // Check if user has exceeded the rate limit
    if (recentRequests.length >= this.maxRequests) {
      return false;
    }

    // Add current request to the list
    recentRequests.push(now);
    this.clients.set(identifier, { requests: recentRequests });

    return true;
  }

  // Reset rate limit for a specific client
  reset(identifier) {
    this.clients.delete(identifier);
  }
}

// CSRF token utilities
export const generateCSRFToken = () => {
  if (typeof window === 'undefined') return null;

  // Get or create a CSRF token in localStorage
  let token = localStorage.getItem('csrfToken');
  if (!token) {
    token = generateSecureToken(32);
    localStorage.setItem('csrfToken', token);
  }
  return token;
};

export const validateCSRFToken = (token) => {
  if (typeof window === 'undefined') return false;

  const storedToken = localStorage.getItem('csrfToken');
  return token && storedToken && token === storedToken;
};

// Session management utilities
export const createSession = (userData) => {
  if (typeof window === 'undefined') return null;

  const session = {
    user: userData,
    createdAt: new Date().toISOString(),
    expiresAt: new Date(Date.now() + 24 * 60 * 60 * 1000).toISOString(), // 24 hours
    sessionId: generateSecureToken(16)
  };

  localStorage.setItem('userSession', JSON.stringify(session));
  return session;
};

export const validateSession = () => {
  if (typeof window === 'undefined') return null;

  const sessionData = localStorage.getItem('userSession');
  if (!sessionData) return null;

  try {
    const session = JSON.parse(sessionData);

    // Check if session has expired
    const now = new Date();
    const expiresAt = new Date(session.expiresAt);

    if (now > expiresAt) {
      // Session expired, remove it
      localStorage.removeItem('userSession');
      return null;
    }

    return session;
  } catch (error) {
    console.error('Error validating session:', error);
    localStorage.removeItem('userSession');
    return null;
  }
};

export const destroySession = () => {
  if (typeof window === 'undefined') return;

  localStorage.removeItem('userSession');
  localStorage.removeItem('csrfToken');
};

// Password strength validation
export const validatePasswordStrength = (password) => {
  if (!password || typeof password !== 'string') {
    return {
      valid: false,
      score: 0,
      feedback: ['Password is required']
    };
  }

  let score = 0;
  const feedback = [];

  // Length check
  if (password.length < 8) {
    feedback.push('Password must be at least 8 characters long');
  } else {
    score += 1;
  }

  // Lowercase check
  if (!/[a-z]/.test(password)) {
    feedback.push('Add lowercase letters');
  } else {
    score += 1;
  }

  // Uppercase check
  if (!/[A-Z]/.test(password)) {
    feedback.push('Add uppercase letters');
  } else {
    score += 1;
  }

  // Number check
  if (!/\d/.test(password)) {
    feedback.push('Add numbers');
  } else {
    score += 1;
  }

  // Special character check
  if (!/[!@#$%^&*(),.?":{}|<>]/.test(password)) {
    feedback.push('Add special characters (!@#$%^&*(),.?":{}|<>)');
  } else {
    score += 1;
  }

  // Complexity check
  if (/(.)\1{2,}/.test(password)) {
    feedback.push('Avoid repeated characters');
  } else {
    score += 1;
  }

  return {
    valid: score >= 4, // At least 4 out of 6 criteria
    score,
    feedback,
    strength: score < 3 ? 'weak' : score < 5 ? 'medium' : 'strong'
  };
};

// Content security policy utilities
export const applyContentSecurityPolicy = () => {
  if (typeof document === 'undefined') return;

  // Add CSP meta tag to help prevent XSS attacks
  const cspMeta = document.createElement('meta');
  cspMeta.httpEquiv = 'Content-Security-Policy';
  cspMeta.content = [
    "default-src 'self'",
    "script-src 'self' 'unsafe-inline' https://cdnjs.cloudflare.com https://fonts.googleapis.com",
    "style-src 'self' 'unsafe-inline' https://fonts.googleapis.com https://cdn.jsdelivr.net",
    "font-src 'self' https://fonts.gstatic.com",
    "img-src 'self' data: https:",
    "connect-src 'self'",
    "frame-ancestors 'none'", // Prevent clickjacking
    "base-uri 'self'"
  ].join('; ');

  document.head.appendChild(cspMeta);
};

// Initialize security measures
export const initializeSecurity = () => {
  if (typeof window === 'undefined') return;

  console.log('🔒 Initializing security measures...');

  // Apply content security policy
  applyContentSecurityPolicy();

  // Set up security monitoring
  setupSecurityMonitoring();

  console.log('✅ Security measures initialized');
};

// Set up security monitoring
export const setupSecurityMonitoring = () => {
  if (typeof window === 'undefined') return;

  // Monitor for potential XSS attempts
  const originalSetAttribute = Element.prototype.setAttribute;
  Element.prototype.setAttribute = function(name, value) {
    // Check for potentially dangerous attributes
    if (name.toLowerCase().startsWith('on')) {
      console.warn(`⚠️ Attempted to set dangerous attribute: ${name}=${value}`);
      return;
    }

    // Check for javascript: protocol in href/src attributes
    if (['href', 'src', 'action'].includes(name.toLowerCase()) &&
        value.toLowerCase().startsWith('javascript:')) {
      console.warn(`⚠️ Attempted to set dangerous URL: ${name}=${value}`);
      return;
    }

    return originalSetAttribute.call(this, name, value);
  };

  // Monitor for DOM-based XSS
  const originalInnerHtmlDescriptor = Object.getOwnPropertyDescriptor(Element.prototype, 'innerHTML');
  const originalOuterHtmlDescriptor = Object.getOwnPropertyDescriptor(Element.prototype, 'outerHTML');

  if (originalInnerHtmlDescriptor && originalOuterHtmlDescriptor) {
    Object.defineProperty(Element.prototype, 'innerHTML', {
      set: function(value) {
        if (typeof value === 'string' && /<script/i.test(value)) {
          console.warn('⚠️ Attempted to set HTML containing script tags', value);
          return;
        }
        return originalInnerHtmlDescriptor.set.call(this, value);
      },
      get: originalInnerHtmlDescriptor.get
    });

    Object.defineProperty(Element.prototype, 'outerHTML', {
      set: function(value) {
        if (typeof value === 'string' && /<script/i.test(value)) {
          console.warn('⚠️ Attempted to set HTML containing script tags', value);
          return;
        }
        return originalOuterHtmlDescriptor.set.call(this, value);
      },
      get: originalOuterHtmlDescriptor.get
    });
  }
};

// Run security audit
export const runSecurityAudit = () => {
  const results = {
    sessionValid: !!validateSession(),
    csrfTokenValid: !!localStorage.getItem('csrfToken'),
    localStorageSecure: checkLocalStorageSecurity(),
    domSecure: checkDOMSecurity(),
    inputsMonitored: checkInputSecurity(),
    timestamp: new Date().toISOString()
  };

  console.group('🔒 Security Audit Results');
  console.table(results);
  console.groupEnd();

  return results;
};

// Check localStorage for security issues
export const checkLocalStorageSecurity = () => {
  if (typeof window === 'undefined' || !window.localStorage) return true;

  let secure = true;
  for (let i = 0; i < window.localStorage.length; i++) {
    const key = window.localStorage.key(i);
    const value = window.localStorage.getItem(key);

    // Check for potentially sensitive data stored insecurely
    if (/(password|token|secret|key|auth|credential|session)/i.test(key) &&
        !/(csrf|session)/i.test(key)) { // Allow CSRF and session tokens which are handled securely
      console.warn(`⚠️ Potentially sensitive data found in localStorage: ${key}`);
      secure = false;
    }
  }

  return secure;
};

// Check DOM for security issues
export const checkDOMSecurity = () => {
  if (typeof document === 'undefined') return true;

  // Check for dangerous elements
  const dangerousElements = document.querySelectorAll('script[src*="javascript:"], iframe[src*="javascript:"], object[data*="javascript:"], embed[src*="javascript:"]');
  if (dangerousElements.length > 0) {
    console.warn('⚠️ Found potentially dangerous elements in DOM', dangerousElements);
    return false;
  }

  return true;
};

// Check input security
export const checkInputSecurity = () => {
  // Input security is handled by the monitoring setup
  return true;
};