import React, { useState, useEffect } from 'react';

const NavbarAuth = () => {
  const [authState, setAuthState] = useState({
    isAuthenticated: false,
    user: null,
    loading: true
  });

  // Check if we're in browser environment before accessing localStorage
  if (typeof window === 'undefined') {
    // In SSR, return placeholder to avoid errors
    return null;
  }

  useEffect(() => {
    try {
      // Check authentication status from localStorage
      const currentUser = localStorage.getItem('currentUser');
      const authToken = localStorage.getItem('authToken');

      if (currentUser && authToken) {
        const user = JSON.parse(currentUser);
        setAuthState({
          isAuthenticated: true,
          user: user,
          loading: false
        });
      } else {
        setAuthState({
          isAuthenticated: false,
          user: null,
          loading: false
        });
      }
    } catch (error) {
      console.error('Error reading auth state from localStorage:', error);
      setAuthState({
        isAuthenticated: false,
        user: null,
        loading: false
      });
    }
  }, []);

  const handleLogout = () => {
    try {
      // Call backend logout if token exists
      const token = localStorage.getItem('authToken');
      if (token) {
        // Use environment variable or default for backend URL
        const backendUrl = (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
          ? process.env.REACT_APP_BACKEND_URL
          : 'http://localhost:8005';
        // Make async call but don't wait for it to avoid blocking UI
        fetch(`${backendUrl}/api/auth/logout`, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${token}`,
            'Content-Type': 'application/json',
          },
        }).catch(console.error); // Handle errors silently
      }

      // Clear local state
      localStorage.removeItem('currentUser');
      localStorage.removeItem('userProfile');
      localStorage.removeItem('authToken');

      // Update state to reflect logout
      setAuthState({
        isAuthenticated: false,
        user: null,
        loading: false
      });
    } catch (error) {
      console.error('Error during logout:', error);
    }
  };

  if (authState.loading) {
    return (
      <div className="navbar__item">
        <span>Loading...</span>
      </div>
    );
  }

  if (authState.isAuthenticated && authState.user) {
    return (
      <div className="navbar__item navbar__user-menu">
        <div className="dropdown dropdown--right dropdown--username">
          <button className="button button--secondary dropdown__trigger">
            <span>{authState.user.email}</span>
          </button>
          <ul className="dropdown__menu">
            <li>
              <a className="dropdown__link" href="/profile">
                Profile
              </a>
            </li>
            <li>
              <a className="dropdown__link" href="#" onClick={(e) => {
                e.preventDefault();
                handleLogout();
              }}>
                Logout
              </a>
            </li>
          </ul>
        </div>
      </div>
    );
  }

  return (
    <div className="navbar__item navbar__auth-links">
      <a href="/login" className="button button--primary margin-right--sm">
        Login
      </a>
      <a href="/register" className="button button--secondary">
        Register
      </a>
    </div>
  );
};

export default NavbarAuth;