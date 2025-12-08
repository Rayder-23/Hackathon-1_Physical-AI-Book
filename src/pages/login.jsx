import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { Login } from '../components/Auth/Login';

function LoginPage() {
  const { isAuthenticated } = useAuth();

  // If already authenticated, redirect to home or dashboard
  if (isAuthenticated) {
    typeof window !== 'undefined' && (window.location.href = '/');
  }

  return (
    <Layout title="Login" description="Login to access protected content">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>Login to Physical AI Book</h1>
            <p>Access protected content, author tools, and personalized features.</p>

            <Login onLoginSuccess={(user) => {
              console.log('Login successful, redirecting...');
              // In a real implementation, you'd redirect to the intended page
              typeof window !== 'undefined' && (window.location.href = '/');
            }} />

            <div className="margin-top--lg">
              <p>Don't have an account? <a href="/register">Register here</a></p>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default LoginPage;