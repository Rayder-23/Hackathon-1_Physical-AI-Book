import React from 'react';
import { useAuth } from '../../contexts/AuthContext';

const NavbarAuth = () => {
  // Check if we're in browser environment before using context
  if (typeof window === 'undefined') {
    // In SSR, return placeholder to avoid context errors
    return null;
  }

  const { user, login, logout, isAuthenticated } = useAuth();

  if (isAuthenticated && user) {
    return (
      <div className="navbar__item navbar__user-menu">
        <div className="dropdown dropdown--right dropdown--username">
          <button className="button button--secondary dropdown__trigger">
            <span>{user.name || user.email}</span>
          </button>
          <ul className="dropdown__menu">
            <li>
              <a className="dropdown__link" href="#" onClick={(e) => {
                e.preventDefault();
                logout();
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