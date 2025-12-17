import React from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { ACCESS_LEVELS, hasAccessToContent } from '../../utils/accessControl';

const ProtectedContent = ({
  children,
  requiredAccessLevel = ACCESS_LEVELS.REGISTERED,
  unauthorizedMessage = "You don't have permission to access this content.",
  showLoginLink = true
}) => {
  // Check if we're in browser environment before using context
  if (typeof window === 'undefined') {
    // In SSR, show content to avoid context errors
    // Authentication will be handled on the client side
    return <div className="protected-content">{children}</div>;
  }

  const { isAuthenticated, user } = useAuth();
  const hasAccess = hasAccessToContent(requiredAccessLevel);

  if (!isAuthenticated) {
    return (
      <div className="protected-content unauthorized">
        <h3>Authentication Required</h3>
        <p>{unauthorizedMessage}</p>
        {showLoginLink && (
          <p>
            <a href="/login" className="button button--primary">Login</a>
          </p>
        )}
      </div>
    );
  }

  if (!hasAccess) {
    return (
      <div className="protected-content unauthorized">
        <h3>Access Denied</h3>
        <p>{unauthorizedMessage}</p>
        <p>Your role: {user?.role || 'user'}</p>
        <p>Required access level: {requiredAccessLevel}</p>
      </div>
    );
  }

  return <div className="protected-content">{children}</div>;
};

export default ProtectedContent;