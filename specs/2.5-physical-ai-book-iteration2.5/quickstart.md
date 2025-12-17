# Quickstart Guide: Authentication, Personalization, and Translation Features

## Prerequisites
- Node.js 18+ installed
- Docusaurus 3.x project set up
- Better-Auth client configured for static hosting

## Installation Steps

### 1. Install Dependencies
```bash
npm install @docusaurus/core better-auth
```

### 2. Set up Authentication Context
Create the authentication context and provider in `src/contexts/AuthContext.jsx`:
```jsx
import React, { createContext, useContext, useState, useEffect } from 'react';

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  // Initialize auth state from localStorage
  useEffect(() => {
    // Check for existing session
    const session = localStorage.getItem('better-auth-session');
    if (session) {
      // Restore user session
      try {
        const sessionData = JSON.parse(session);
        setUser(sessionData.user);
      } catch (e) {
        console.error('Failed to restore session', e);
      }
    }
    setLoading(false);
  }, []);

  const login = async (username, password) => {
    // Implement Better-Auth client login
    if (typeof window !== 'undefined') {
      // Use Better-Auth client login
      setUser({ id: 'mock-user-id', username });
      localStorage.setItem('better-auth-session', JSON.stringify({
        user: { id: 'mock-user-id', username },
        token: 'mock-token'
      }));
    }
  };

  const logout = async () => {
    if (typeof window !== 'undefined') {
      setUser(null);
      localStorage.removeItem('better-auth-session');
    }
  };

  const register = async (username, email, password) => {
    if (typeof window !== 'undefined') {
      // Implement Better-Auth client registration
      setUser({ id: 'mock-user-id', username, email });
      localStorage.setItem('better-auth-session', JSON.stringify({
        user: { id: 'mock-user-id', username, email },
        token: 'mock-token'
      }));
      return { success: true };
    }
  };

  return (
    <AuthContext.Provider value={{ user, login, logout, register, loading }}>
      {children}
    </AuthContext.Provider>
  );
};
```

### 3. Set up Personalization Context
Create `src/contexts/PersonalizationContext.jsx`:
```jsx
import React, { createContext, useContext, useState } from 'react';
import { useAuth } from './AuthContext';

const PersonalizationContext = createContext();

export const usePersonalization = () => {
  const context = useContext(PersonalizationContext);
  if (!context) {
    throw new Error('usePersonalization must be used within a PersonalizationProvider');
  }
  return context;
};

export const PersonalizationProvider = ({ children }) => {
  const { user } = useAuth();
  const [settings, setSettings] = useState({
    difficulty: 'beginner',
    focus: 'mixed'
  });

  const updateSettings = (newSettings) => {
    if (user) {
      setSettings(prev => ({ ...prev, ...newSettings }));
      // Store in localStorage for persistence
      localStorage.setItem(`personalization-${user.id}`, JSON.stringify({
        ...settings,
        ...newSettings
      }));
    }
  };

  return (
    <PersonalizationContext.Provider value={{ settings, updateSettings }}>
      {children}
    </PersonalizationContext.Provider>
  );
};
```

### 4. Set up Translation Context
Create `src/contexts/TranslationContext.jsx`:
```jsx
import React, { createContext, useContext, useState, useEffect } from 'react';

const TranslationContext = createContext();

export const useTranslation = () => {
  const context = useContext(TranslationContext);
  if (!context) {
    throw new Error('useTranslation must be used within a TranslationProvider');
  }
  return context;
};

export const TranslationProvider = ({ children }) => {
  const [language, setLanguage] = useState('en');
  const [translations, setTranslations] = useState({});

  // Load Urdu translations
  useEffect(() => {
    const loadTranslations = async () => {
      if (typeof window !== 'undefined' && language === 'ur') {
        try {
          const urduTranslations = await import('../../data/ur/book.json');
          setTranslations(urduTranslations.default);
        } catch (e) {
          console.error('Failed to load Urdu translations', e);
        }
      }
    };
    loadTranslations();
  }, [language]);

  const toggleLanguage = () => {
    setLanguage(prev => prev === 'en' ? 'ur' : 'en');
  };

  return (
    <TranslationContext.Provider value={{ language, toggleLanguage, translations }}>
      {children}
    </TranslationContext.Provider>
  );
};
```

### 5. Create Root.js Wrapper
Create `src/theme/Root.js`:
```jsx
import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import { PersonalizationProvider } from '../contexts/PersonalizationContext';
import { TranslationProvider } from '../contexts/TranslationContext';

const SecurityProvider = ({ children }) => {
  if (typeof window !== 'undefined') {
    return (
      <AuthProvider>
        <PersonalizationProvider>
          <TranslationProvider>
            {children}
          </TranslationProvider>
        </PersonalizationProvider>
      </AuthProvider>
    );
  }
  return <>{children}</>;
};

export default function Root({ children }) {
  return <SecurityProvider>{children}</SecurityProvider>;
}
```

### 6. Create Login Page
Create `src/pages/login.jsx`:
```jsx
import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useHistory, useLocation } from '@docusaurus/router';

function LoginPage() {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { login } = useAuth();
  const history = useHistory();
  const location = useLocation();

  // Get redirect URL from query params or default to home
  const { from } = location.state || { from: { pathname: "/" } };

  const handleSubmit = async (e) => {
    e.preventDefault();
    try {
      await login(username, password);
      history.replace(from);
    } catch (err) {
      setError('Login failed. Please check your credentials.');
    }
  };

  if (typeof window === 'undefined') {
    return <div>Loading...</div>;
  }

  return (
    <div style={{ padding: '2rem', maxWidth: '400px', margin: '0 auto' }}>
      <h1>Login</h1>
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <form onSubmit={handleSubmit}>
        <div style={{ marginBottom: '1rem' }}>
          <label>Username:</label>
          <input
            type="text"
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            style={{ width: '100%', padding: '0.5rem' }}
            required
          />
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label>Password:</label>
          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            style={{ width: '100%', padding: '0.5rem' }}
            required
          />
        </div>
        <button type="submit" style={{ padding: '0.5rem 1rem' }}>Login</button>
      </form>
      <p>
        Don't have an account? <a href="/register">Register here</a>
      </p>
    </div>
  );
}

export default LoginPage;
```

### 7. Create Register Page
Create `src/pages/register.jsx`:
```jsx
import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

function RegisterPage() {
  const [username, setUsername] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [error, setError] = useState('');
  const { register } = useAuth();
  const history = useHistory();

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    try {
      const result = await register(username, email, password);
      if (result.success) {
        history.push('/login');
      }
    } catch (err) {
      setError('Registration failed. Please try again.');
    }
  };

  if (typeof window === 'undefined') {
    return <div>Loading...</div>;
  }

  return (
    <div style={{ padding: '2rem', maxWidth: '400px', margin: '0 auto' }}>
      <h1>Register</h1>
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <form onSubmit={handleSubmit}>
        <div style={{ marginBottom: '1rem' }}>
          <label>Username:</label>
          <input
            type="text"
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            style={{ width: '100%', padding: '0.5rem' }}
            required
          />
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label>Email (optional):</label>
          <input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            style={{ width: '100%', padding: '0.5rem' }}
          />
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label>Password:</label>
          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            style={{ width: '100%', padding: '0.5rem' }}
            required
          />
        </div>
        <div style={{ marginBottom: '1rem' }}>
          <label>Confirm Password:</label>
          <input
            type="password"
            value={confirmPassword}
            onChange={(e) => setConfirmPassword(e.target.value)}
            style={{ width: '100%', padding: '0.5rem' }}
            required
          />
        </div>
        <button type="submit" style={{ padding: '0.5rem 1rem' }}>Register</button>
      </form>
      <p>
        Already have an account? <a href="/login">Login here</a>
      </p>
    </div>
  );
}

export default RegisterPage;
```

### 8. Create Translation Toggle Component
Create `src/components/Translation/Toggle.jsx`:
```jsx
import React from 'react';
import { useTranslation } from '../../contexts/TranslationContext';

function TranslationToggle() {
  if (typeof window === 'undefined') {
    return null;
  }

  const { language, toggleLanguage } = useTranslation();

  return (
    <button
      onClick={toggleLanguage}
      style={{
        padding: '0.5rem 1rem',
        margin: '0 0.5rem',
        border: '1px solid #ccc',
        borderRadius: '4px',
        cursor: 'pointer'
      }}
    >
      {language === 'en' ? 'اردو' : 'English'}
    </button>
  );
}

export default TranslationToggle;
```

### 9. Create Personalization Toggle Component
Create `src/components/Personalization/Toggle.jsx`:
```jsx
import React from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';

function PersonalizationToggle() {
  if (typeof window === 'undefined') {
    return null;
  }

  const { settings, updateSettings } = usePersonalization();

  const handleDifficultyChange = (e) => {
    updateSettings({ difficulty: e.target.value });
  };

  const handleFocusChange = (e) => {
    updateSettings({ focus: e.target.value });
  };

  return (
    <div style={{ display: 'flex', gap: '1rem', alignItems: 'center' }}>
      <select
        value={settings.difficulty}
        onChange={handleDifficultyChange}
        style={{ padding: '0.25rem' }}
      >
        <option value="beginner">Beginner</option>
        <option value="intermediate">Intermediate</option>
        <option value="advanced">Advanced</option>
      </select>

      <select
        value={settings.focus}
        onChange={handleFocusChange}
        style={{ padding: '0.25rem' }}
      >
        <option value="mixed">Mixed</option>
        <option value="software">Software</option>
        <option value="hardware">Hardware</option>
      </select>
    </div>
  );
}

export default PersonalizationToggle;
```

### 10. Create Translatable Content Component
Create `src/theme/MDXComponents/TranslatableContent.jsx`:
```jsx
import React from 'react';
import { useTranslation } from '../../contexts/TranslationContext';

function TranslatableContent({ children, id }) {
  if (typeof window === 'undefined') {
    return <>{children}</>;
  }

  const { language, translations } = useTranslation();

  if (language === 'ur' && id && translations[id]) {
    return <>{translations[id]}</>;
  }

  return <>{children}</>;
}

export default TranslatableContent;
```

### 11. Create Personalized Content Component
Create `src/theme/MDXComponents/PersonalizedContent.jsx`:
```jsx
import React from 'react';
import { usePersonalization } from '../../contexts/PersonalizationContext';

function PersonalizedContent({ children, difficulty, focus }) {
  if (typeof window === 'undefined') {
    return null;
  }

  const { settings } = usePersonalization();

  // Show content if it matches user's difficulty or focus settings
  const matchesDifficulty = !difficulty || settings.difficulty === difficulty;
  const matchesFocus = !focus || settings.focus === focus;

  if (matchesDifficulty && matchesFocus) {
    return <>{children}</>;
  }

  return null;
}

export default PersonalizedContent;
```

### 12. Create Protected Content Component
Create `src/theme/MDXComponents/ProtectedContent.jsx`:
```jsx
import React from 'react';
import { useAuth } from '../../contexts/AuthContext';

function ProtectedContent({ children, requiredRole }) {
  if (typeof window === 'undefined') {
    return null;
  }

  const { user } = useAuth();

  if (user) {
    return <>{children}</>;
  }

  return <p>Please log in to view this content.</p>;
}

export default ProtectedContent;
```

## Testing the Implementation

### 1. Verify SSR Safety
Run the build command to ensure no SSR errors:
```bash
npm run build
```

### 2. Test Authentication Flow
1. Navigate to `/register` to create an account
2. Navigate to `/login` to log in
3. Verify session persists across page refreshes
4. Test logout functionality

### 3. Test Translation Toggle
1. Verify translation toggle appears for authenticated users
2. Test switching between English and Urdu
3. Verify content updates instantly

### 4. Test Personalization
1. Verify personalization controls appear for authenticated users
2. Test changing difficulty and focus settings
3. Verify content filters based on settings

## Deployment to GitHub Pages

The implementation is designed for static hosting compatibility:
1. All functionality is client-side
2. No server dependencies
3. SSR-safe patterns implemented
4. Build process produces static files only