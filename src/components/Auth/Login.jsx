import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';

const Login = ({ onLoginSuccess, onSwitchToRegister }) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { login, isAuthenticated, user, logout } = useAuth();

  // If user is already logged in, show a message
  if (isAuthenticated && user) {
    return (
      <div className="auth-container">
        <div className="auth-form">
          <h2>Already Signed In</h2>
          <div className="success-message">
            You are already signed in as {user.email}.
            <br />
            <a href="/Hackathon-1_Physical-AI-Book/" style={{ color: '#2563eb', textDecoration: 'underline' }}>Go to homepage</a> or
            <a href="#"
               onClick={(e) => {
                 e.preventDefault();
                 if (window.confirm('Would you like to logout of your current account?')) {
                   logout();
                 }
               }}
               style={{ color: '#2563eb', textDecoration: 'underline', marginLeft: '4px' }}>
              logout
            </a>.
          </div>
        </div>
      </div>
    );
  }

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const result = await login(email, password);
      if (result.success) {
        if (onLoginSuccess) {
          onLoginSuccess(result.user, result.profile);
        }
      } else {
        setError(result.error || 'Login failed. Please try again.');
      }
    } catch (err) {
      console.error('Login error:', err);
      // Check if the error is due to HTML response instead of JSON
      if (err.message && err.message.includes('Unexpected token') && err.message.includes('<!DOCTYPE')) {
        setError('Server error: Unable to process request. Please check that the backend server is running.');
      } else {
        setError(err.message || 'Login failed. Please try again.');
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Login</h2>
        {error && <div className="error-message">{error}</div>}

        <form onSubmit={handleSubmit}>
          <div className="form-group">
            <label htmlFor="email">Email:</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
              disabled={loading}
            />
          </div>

          <div className="form-group">
            <label htmlFor="password">Password:</label>
            <div className="password-input-container">
              <input
                type={showPassword ? "text" : "password"}
                id="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                disabled={loading}
              />
              <button
                type="button"
                className="password-toggle-button"
                onClick={() => setShowPassword(!showPassword)}
                disabled={loading}
              >
                {showPassword ? 'Hide' : 'Show'}
              </button>
            </div>
          </div>

          <button type="submit" disabled={loading} className="auth-button">
            {loading ? 'Logging in...' : 'Login'}
          </button>
        </form>

        <div className="auth-link">
          <p>Don't have an account?{' '}
            <a href="#" onClick={(e) => {
              e.preventDefault();
              if (onSwitchToRegister) {
                onSwitchToRegister();
              }
            }}>
              Register here
            </a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default Login;
export { Login };