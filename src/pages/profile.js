import React from 'react';
import Layout from '@theme/Layout';
import Profile from '../components/Auth/Profile';

function ProfilePage() {
  return (
    <Layout title="Profile" description="Manage your profile">
      <div style={{ padding: '2rem', maxWidth: '600px', margin: '0 auto' }}>
        <Profile />
      </div>
    </Layout>
  );
}

export default ProfilePage;