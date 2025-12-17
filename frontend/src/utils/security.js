// Security utilities for Physical AI Book
// Implements client-side security measures for static hosting

// Input sanitization utilities
export const sanitizeInput = (input) => {
  if (typeof input !== 'string') return input;

  // Remove potentially dangerous characters/sequences
  return input
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '') // Remove script tags
    .replace(/javascript:/gi, '') // Remove javascript: protocols
    .replace(/on\w+="[^"]*"/gi, '') // Remove event handlers
    .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '') // Remove iframes
    .trim();
};

// Validate email format
export const validateEmail = (email) => {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
};

// Validate password strength
export const validatePassword = (password) => {
  // At least 8 characters, one uppercase, one lowercase, one number, one special character
  const passwordRegex = /^(?=.*[a-z])(?=.*[A-Z])(?=.*\d)(?=.*[@$!%*?&])[A-Za-z\d@$!%*?&]{8,}$/;
  return passwordRegex.test(password);
};

// Sanitize user profile data
export const sanitizeUserProfile = (profileData) => {
  if (!profileData || typeof profileData !== 'object') return {};

  return {
    name: sanitizeInput(profileData.name || ''),
    email: sanitizeInput(profileData.email || ''),
    role: sanitizeInput(profileData.role || 'user'),
    background: sanitizeInput(profileData.background || ''),
    experienceLevel: sanitizeInput(profileData.experienceLevel || 'intermediate'),
    preferences: profileData.preferences || {},
    createdAt: profileData.createdAt || new Date().toISOString()
  };
};

// Check content access based on user role and content restrictions
export const checkContentAccess = (user, contentMetadata) => {
  if (!user) {
    // If no user, only allow public content
    return (contentMetadata && contentMetadata.accessLevel === 'public') ||
           !contentMetadata ||
           contentMetadata.accessLevel === undefined;
  }

  // For registered users, allow public and registered content
  if (contentMetadata &&
      (contentMetadata.accessLevel === 'public' ||
       contentMetadata.accessLevel === 'registered')) {
    return true;
  }

  // For authors, allow author-specific content
  if (user.role === 'author' &&
      (contentMetadata.accessLevel === 'author' ||
       contentMetadata.accessLevel === 'admin')) {
    return true;
  }

  // For admins, allow admin content
  if (user.role === 'admin' && contentMetadata.accessLevel === 'admin') {
    return true;
  }

  // Default to no access if no conditions match
  return false;
};

// Validate content before saving to localStorage
export const validateContentForStorage = (content, contentType = 'generic') => {
  try {
    // Check for potential XSS content
    if (typeof content === 'string') {
      // Check for script tags or other dangerous content
      if (/<script/i.test(content) || /javascript:/i.test(content) || /on\w+="[^"]*"/i.test(content)) {
        return {
          valid: false,
          error: 'Content contains potentially dangerous elements'
        };
      }

      // Check content length limits
      const maxLength = {
        'profile': 10000,
        'comment': 1000,
        'generic': 50000
      }[contentType] || 50000;

      if (content.length > maxLength) {
        return {
          valid: false,
          error: `Content exceeds maximum length of ${maxLength} characters`
        };
      }
    }

    return {
      valid: true,
      content: content
    };
  } catch (error) {
    return {
      valid: false,
      error: error.message
    };
  }
};

// Hash function for client-side obfuscation (not cryptographic security)
export const simpleHash = (str) => {
  let hash = 0;
  for (let i = 0; i < str.length; i++) {
    const char = str.charCodeAt(i);
    hash = ((hash << 5) - hash) + char;
    hash = hash & hash; // Convert to 32bit integer
  }
  return Math.abs(hash).toString();
};

// Generate secure token for client-side session tracking
export const generateSecureToken = (length = 32) => {
  if (typeof window !== 'undefined' && window.crypto && window.crypto.getRandomValues) {
    // Use Web Crypto API if available
    const array = new Uint8Array(length);
    window.crypto.getRandomValues(array);
    return Array.from(array, byte => byte.toString(16).padStart(2, '0')).join('');
  } else {
    // Fallback to Math.random (less secure)
    let result = '';
    const characters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    for (let i = 0; i < length; i++) {
      result += characters.charAt(Math.floor(Math.random() * characters.length));
    }
    return result;
  }
};

// Encrypt sensitive data for localStorage (obfuscation, not encryption)
export const encryptLocalStorageData = (data) => {
  try {
    if (typeof window === 'undefined') return JSON.stringify(data);

    // Simple obfuscation by combining with a session-based key
    const sessionKey = window.location.hostname + new Date().getTime().toString().slice(0, -3);
    const jsonString = JSON.stringify(data);

    // XOR with session key for basic obfuscation
    let result = '';
    for (let i = 0; i < jsonString.length; i++) {
      result += String.fromCharCode(jsonString.charCodeAt(i) ^ sessionKey.charCodeAt(i % sessionKey.length));
    }

    return btoa(encodeURIComponent(result)); // Base64 encode for storage
  } catch (error) {
    console.error('Error encrypting localStorage data:', error);
    return JSON.stringify(data); // Fallback to plain JSON
  }
};

// Decrypt sensitive data from localStorage
export const decryptLocalStorageData = (encryptedData) => {
  try {
    if (typeof window === 'undefined' || !encryptedData) return JSON.parse(encryptedData || '{}');

    // Decode and decrypt the data
    const decoded = decodeURIComponent(atob(encryptedData));

    // Reverse XOR with a predictable key (this is obfuscation, not strong encryption)
    const sessionKey = window.location.hostname + new Date().getTime().toString().slice(0, -3);
    let result = '';
    for (let i = 0; i < decoded.length; i++) {
      result += String.fromCharCode(decoded.charCodeAt(i) ^ sessionKey.charCodeAt(i % sessionKey.length));
    }

    return JSON.parse(result);
  } catch (error) {
    console.error('Error decrypting localStorage data:', error);
    return JSON.parse(encryptedData || '{}'); // Fallback to parsing directly
  }
};

// Validate authentication token (client-side only)
export const validateAuthToken = (token) => {
  if (!token || typeof token !== 'string') return false;

  // Basic validation - check if it looks like a token
  // In a real implementation, this would verify with the server
  return token.length >= 16 && /^[a-zA-Z0-9-_]+$/.test(token);
};

// Rate limiting for client-side actions
export class RateLimiter {
  constructor(windowMs = 60000, maxRequests = 5) {
    this.windowMs = windowMs;
    this.maxRequests = maxRequests;
    this.requests = new Map();
  }

  isAllowed(identifier) {
    const now = Date.now();
    const userRequests = this.requests.get(identifier) || [];

    // Remove old requests outside the window
    const recentRequests = userRequests.filter(timestamp => now - timestamp < this.windowMs);

    if (recentRequests.length >= this.maxRequests) {
      return false; // Rate limit exceeded
    }

    // Add current request
    recentRequests.push(now);
    this.requests.set(identifier, recentRequests);

    return true;
  }
}

// Security headers for static hosting (simulated)
export const applySecurityHeaders = () => {
  // In a real implementation, these would be set in the server configuration
  // For static hosting, we simulate this by setting appropriate meta tags

  if (typeof window === 'undefined' || typeof document === 'undefined') return;

  // Add security-related meta tags
  const securityMetaTags = [
    { name: 'referrer', content: 'strict-origin-when-cross-origin' },
    { name: 'content-security-policy', content: "default-src 'self'; script-src 'self' 'unsafe-inline' https://cdnjs.cloudflare.com https://fonts.googleapis.com; style-src 'self' 'unsafe-inline' https://fonts.googleapis.com https://cdn.jsdelivr.net; font-src 'self' https://fonts.gstatic.com; img-src 'self' data: https:;" }
  ];

  securityMetaTags.forEach(tag => {
    let meta = document.querySelector(`meta[name="${tag.name}"]`);
    if (!meta) {
      meta = document.createElement('meta');
      meta.name = tag.name;
      document.head.appendChild(meta);
    }
    meta.content = tag.content;
  });
};

// Initialize security measures
export const initializeSecurity = () => {
  if (typeof window === 'undefined') return;

  console.log('🔒 Initializing security measures...');

  // Apply security headers
  applySecurityHeaders();

  // Set up security monitoring
  setupSecurityMonitoring();

  console.log('✅ Security measures initialized');
};

// Set up security monitoring
export const setupSecurityMonitoring = () => {
  if (typeof window === 'undefined') return;

  // Monitor for XSS attempts
  const originalAddEventListener = EventTarget.prototype.addEventListener;
  EventTarget.prototype.addEventListener = function(type, listener, options) {
    // Add security checks for input events
    if (type === 'input' || type === 'change') {
      const securedListener = function(event) {
        if (event.target && event.target.value) {
          // Sanitize input if needed
          event.target.value = sanitizeInput(event.target.value);
        }
        return listener.call(this, event);
      };
      return originalAddEventListener.call(this, type, securedListener, options);
    }
    return originalAddEventListener.call(this, type, listener, options);
  };
};

// Security audit utility
export const runSecurityAudit = () => {
  const results = {
    localStorage: checkLocalStorageSecurity(),
    sessionStorage: checkSessionStorageSecurity(),
    cookies: checkCookieSecurity(),
    dom: checkDOMSecurity(),
    inputs: checkInputSecurity()
  };

  console.table(results);
  return results;
};

// Check localStorage for sensitive data
export const checkLocalStorageSecurity = () => {
  if (typeof window === 'undefined' || !window.localStorage) return true;

  let hasSensitiveData = false;
  for (let i = 0; i < window.localStorage.length; i++) {
    const key = window.localStorage.key(i);
    const value = window.localStorage.getItem(key);

    // Check if the key or value contains sensitive information
    if (/password|token|secret|key|auth|credential/i.test(key) ||
        /password|token|secret|key|auth|credential/i.test(value)) {
      hasSensitiveData = true;
      console.warn(`⚠️ Potential sensitive data in localStorage under key: ${key}`);
    }
  }

  return !hasSensitiveData;
};

// Check for DOM security issues
export const checkDOMSecurity = () => {
  if (typeof document === 'undefined') return true;

  // Check for potentially dangerous elements
  const dangerousElements = [
    ...document.querySelectorAll('script:not([src*="mathjax"])'),
    ...document.querySelectorAll('iframe'),
    ...document.querySelectorAll('object'),
    ...document.querySelectorAll('embed')
  ].filter(el =>
    !el.src || !el.src.includes('trusted-domain') // Add trusted domains as needed
  );

  if (dangerousElements.length > 0) {
    console.warn('⚠️ Found potentially dangerous elements in DOM:', dangerousElements);
    return false;
  }

  return true;
};

// Check input security
export const checkInputSecurity = () => {
  if (typeof document === 'undefined') return true;

  // Check for input fields without proper validation
  const inputs = document.querySelectorAll('input[type="text"], input[type="email"], input[type="password"], textarea');
  let hasInsecureInputs = false;

  inputs.forEach(input => {
    // Check if input has proper validation attributes
    if (!input.hasAttribute('pattern') &&
        !input.hasAttribute('minlength') &&
        !input.className.includes('secure-input')) {
      console.warn(`⚠️ Input field may lack proper validation:`, input.name || input.id || input.placeholder);
      hasInsecureInputs = true;
    }
  });

  return !hasInsecureInputs;
};

// Check cookie security (for completeness, though not used in static hosting)
export const checkCookieSecurity = () => {
  if (typeof document === 'undefined') return true;

  // In static hosting, we don't typically use cookies, but we check anyway
  const cookies = document.cookie.split(';');
  let hasInsecureCookies = false;

  for (const cookie of cookies) {
    const [name, value] = cookie.trim().split('=');
    if (name && value) {
      console.warn(`⚠️ Cookie found in static site: ${name}. Cookies should be avoided in static hosting.`);
      hasInsecureCookies = true;
    }
  }

  return !hasInsecureCookies;
};

// Check sessionStorage security
export const checkSessionStorageSecurity = () => {
  if (typeof window === 'undefined' || !window.sessionStorage) return true;

  let hasSensitiveData = false;
  for (let i = 0; i < window.sessionStorage.length; i++) {
    const key = window.sessionStorage.key(i);
    const value = window.sessionStorage.getItem(key);

    // Check if the key or value contains sensitive information
    if (/password|token|secret|key|auth|credential/i.test(key) ||
        /password|token|secret|key|auth|credential/i.test(value)) {
      hasSensitiveData = true;
      console.warn(`⚠️ Potential sensitive data in sessionStorage under key: ${key}`);
    }
  }

  return !hasSensitiveData;
};