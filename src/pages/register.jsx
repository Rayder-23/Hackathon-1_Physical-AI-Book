import React from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { Register } from '../components/Auth/Register';

function RegisterPage() {
  const { isAuthenticated } = useAuth();

  // If already authenticated, redirect to home or dashboard
  if (isAuthenticated) {
    if (typeof window !== 'undefined') {
      window.location.href = '/Hackathon-1_Physical-AI-Book/';
    }
    return null; // Prevent rendering during redirect
  }

  return (
    <Layout title="Register" description="Create an account to access protected content">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1>Register for Physical AI Book</h1>
            <p>Create an account to access protected content, author tools, and personalized features.</p>

            <Register onRegisterSuccess={(user) => {
              console.log('Registration successful, redirecting...', user);
              // In a real implementation, you'd redirect to the intended page
              if (typeof window !== 'undefined') {
                // Redirect to homepage with proper base URL
                window.location.href = window.location.origin + '/Hackathon-1_Physical-AI-Book/';
              }
            }} />

            <div className="margin-top--lg">
              <p>Already have an account? <a href="login">Login here</a></p>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default RegisterPage;