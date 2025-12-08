import React from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import { ACCESS_LEVELS, hasAccessToContent } from '@site/src/utils/accessControl';

const ProtectedContent = ({
  children,
  requiredAccessLevel = ACCESS_LEVELS.REGISTERED,
  unauthorizedMessage = "You don't have permission to access this content.",
  showLoginLink = true
}) => {
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