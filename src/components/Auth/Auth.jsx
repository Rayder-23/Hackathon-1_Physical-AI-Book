import React, { useState } from 'react';
import Login from './Login';
import Register from './Register';

const Auth = ({ onAuthSuccess }) => {
  const [currentView, setCurrentView] = useState('login'); // 'login' or 'register'

  const handleAuthSuccess = (user, profile) => {
    if (onAuthSuccess) {
      onAuthSuccess(user, profile);
    }
  };

  const switchToLogin = () => {
    setCurrentView('login');
  };

  const switchToRegister = () => {
    setCurrentView('register');
  };

  return (
    <div>
      {currentView === 'login' ? (
        <Login
          onLoginSuccess={handleAuthSuccess}
          onSwitchToRegister={switchToRegister}
        />
      ) : (
        <Register
          onRegisterSuccess={handleAuthSuccess}
          onSwitchToLogin={switchToLogin}
        />
      )}
    </div>
  );
};

export default Auth;
export { Auth };