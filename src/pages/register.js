import React from 'react';
import Layout from '@theme/Layout';
import Register from '../components/Auth/Register';

function RegisterPage() {
  const handleRegisterSuccess = (user, profile) => {
    // Redirect to home or previous page after registration
    window.location.href = '/Hackathon-1_Physical-AI-Book/';
  };

  return (
    <Layout title="Register" description="Create a new account">
      <div style={{ padding: '2rem', maxWidth: '600px', margin: '0 auto' }}>
        <Register onRegisterSuccess={handleRegisterSuccess} />
      </div>
    </Layout>
  );
}

export default RegisterPage;