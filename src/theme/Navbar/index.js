import React, { useEffect, useState } from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import { useAuth } from '../../contexts/AuthContext';

// Create a wrapper component that adds auth functionality to the navbar
const NavbarWrapper = (props) => {
  // Check if we're in browser environment before using context
  if (typeof window === 'undefined') {
    // In SSR, just render the original navbar without auth checks
    return <OriginalNavbar {...props} />;
  }

  const { user, login, logout, isAuthenticated } = useAuth();
  const [isClient, setIsClient] = useState(false);
  const [logoutTriggered, setLogoutTriggered] = useState(false);

  useEffect(() => {
    setIsClient(true);

    // Listen for logout events triggered from the dropdown
    const handleLogout = () => {
      logout();
      setLogoutTriggered(prev => !prev); // Trigger re-render
    };

    window.addEventListener('navbarLogout', handleLogout);

    return () => {
      window.removeEventListener('navbarLogout', handleLogout);
    };
  }, [logout]);

  if (!isClient) {
    return <OriginalNavbar {...props} />;
  }

  // Clone the original items and add auth items if they don't exist already
  const originalItems = props.items || [];

  // Check if auth items already exist to avoid duplicates
  const hasAuthItems = originalItems.some(item =>
    item.href === '/login' || item.href === '/register'
  );

  let newItems = [...originalItems];

  if (!hasAuthItems) {
    // Add auth items to the right side
    if (!isAuthenticated) {
      newItems = [
        ...originalItems,
        {
          href: '/login',
          label: 'Login',
          position: 'right',
        },
        {
          href: '/register',
          label: 'Register',
          position: 'right',
        }
      ];
    } else if (user) {
      // Add user dropdown - use HTML to handle logout click
      newItems = [
        ...originalItems,
        {
          type: 'dropdown',
          label: user.name || user.email,
          position: 'right',
          items: [
            {
              html: `<a href="#" onclick="event.preventDefault(); window.dispatchEvent(new CustomEvent('navbarLogout'));">Logout</a>`
            }
          ]
        }
      ];
    }
  }

  return <OriginalNavbar {...props} items={newItems} />;
};

export default NavbarWrapper;