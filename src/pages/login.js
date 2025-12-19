import React from 'react';
import Layout from '@theme/Layout';
import Login from '../components/Auth/Login';

function LoginPage() {
  const handleLoginSuccess = (user, profile) => {
    // Redirect to home or previous page after login
    window.location.href = '/Hackathon-1_Physical-AI-Book/';
  };

  return (
    <Layout title="Login" description="Login to your account">
      <div style={{ padding: '2rem', maxWidth: '600px', margin: '0 auto' }}>
        <Login onLoginSuccess={handleLoginSuccess} />
      </div>
    </Layout>
  );
}

export default LoginPage;