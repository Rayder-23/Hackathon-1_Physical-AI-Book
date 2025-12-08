// Access control utilities for protected content
import { getCurrentUser } from '../config/auth';

// Define user roles and permissions
export const USER_ROLES = {
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
export const isAuthenticated = async () => {
  try {
    const user = await getCurrentUser();
    return !!user;
  } catch (error) {
    console.error('Error checking authentication:', error);
    return false;
  }
};

// Get current user's role
export const getUserRole = () => {
  try {
    const user = getCurrentUser();
    return user?.role || USER_ROLES.USER;
  } catch (error) {
    console.error('Error getting user role:', error);
    return USER_ROLES.USER;
  }
};

// Check if user has a specific role
export const hasRole = (requiredRole) => {
  const userRole = getUserRole();
  // Admins have access to everything
  if (userRole === USER_ROLES.ADMIN) return true;
  // Check specific role
  return userRole === requiredRole;
};

// Check if user has access to specific content based on access level
export const hasAccessToContent = (requiredAccessLevel) => {
  // Public content is accessible to everyone
  if (requiredAccessLevel === ACCESS_LEVELS.PUBLIC) return true;

  // Check authentication first
  const user = getCurrentUser();
  if (!user) return false;

  // Registered users can access registered content
  if (requiredAccessLevel === ACCESS_LEVELS.REGISTERED) return true;

  // Check role-based access
  const userRole = user.role || USER_ROLES.USER;

  switch (requiredAccessLevel) {
    case ACCESS_LEVELS.AUTHOR:
      return userRole === USER_ROLES.AUTHOR || userRole === USER_ROLES.ADMIN;
    case ACCESS_LEVELS.ADMIN:
      return userRole === USER_ROLES.ADMIN;
    default:
      return false;
  }
};

// Higher-order function to create access guard components
export const withAccessGuard = (WrappedComponent, requiredAccessLevel) => {
  return function AccessGuardComponent(props) {
    const hasAccess = hasAccessToContent(requiredAccessLevel);

    if (!hasAccess) {
      return (
        <div className="access-denied">
          <h2>Access Denied</h2>
          <p>You don't have permission to access this content.</p>
          <a href="/login">Please log in</a>
        </div>
      );
    }

    return <WrappedComponent {...props} />;
  };
};

// Check access and return appropriate content
export const checkAccessAndRender = (requiredAccessLevel, renderContent, renderUnauthorized = null) => {
  const hasAccess = hasAccessToContent(requiredAccessLevel);

  if (hasAccess) {
    return renderContent();
  }

  if (renderUnauthorized) {
    return renderUnauthorized();
  }

  // Default unauthorized content
  return (
    <div className="access-denied">
      <h2>Access Denied</h2>
      <p>You don't have permission to access this content.</p>
      <a href="/login">Please log in</a>
    </div>
  );
};

// Middleware-style function for access control
export const requireAccess = async (requiredAccessLevel, redirectTo = '/login') => {
  const isAuthenticatedUser = await isAuthenticated();
  if (!isAuthenticatedUser) {
    // In a Docusaurus context, we'd typically redirect using navigation
    // For now, we'll return a flag indicating access status
    return { hasAccess: false, redirect: redirectTo };
  }

  const hasAccess = hasAccessToContent(requiredAccessLevel);
  if (!hasAccess) {
    return { hasAccess: false, redirect: '/unauthorized' };
  }

  return { hasAccess: true };
};

// Initialize access control system
export const initializeAccessControl = () => {
  // Set up any necessary event listeners or initialization
  // For static sites, this might involve checking URL parameters or session state
  console.log('Access control system initialized');

  // Check if user has valid session on initialization
  return isAuthenticated();
};