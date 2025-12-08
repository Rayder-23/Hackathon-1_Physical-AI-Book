import React from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { Redirect } from '@docusaurus/router';

const ProtectedRoute = ({ children, requiredRole = 'user', redirectTo = '/login' }) => {
  const { isAuthenticated, loading, user } = useAuth();

  // Show loading state while checking authentication
  if (loading) {
    return (
      <div className="protected-route-loading">
        <p>Loading...</p>
      </div>
    );
  }

  // If not authenticated, redirect to login
  if (!isAuthenticated) {
    return <Redirect to={redirectTo} />;
  }

  // Check if user has required role (for more granular access control)
  if (requiredRole && user?.role !== requiredRole && requiredRole !== 'user') {
    return (
      <div className="protected-route-unauthorized">
        <h2>Access Denied</h2>
        <p>You don't have permission to access this resource.</p>
        <a href="/">Go back to home</a>
      </div>
    );
  }

  // User is authenticated and authorized, render children
  return <>{children}</>;
};

export default ProtectedRoute;
export { ProtectedRoute };