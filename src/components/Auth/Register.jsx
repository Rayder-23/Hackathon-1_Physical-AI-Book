import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';

const Register = ({ onRegisterSuccess, onSwitchToLogin }) => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [showPassword, setShowPassword] = useState(false);
  const [softwareExperience, setSoftwareExperience] = useState('beginner');
  const [hardwareExperience, setHardwareExperience] = useState('beginner');
  const [backgroundPreference, setBackgroundPreference] = useState('software');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { register, isAuthenticated, user, logout } = useAuth();

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

    // Validate password strength
    const passwordRegex = /^(?=.*[a-z])(?=.*[A-Z])(?=.*\d).{8,}$/;
    if (!passwordRegex.test(password)) {
      setError('Password must be at least 8 characters long and contain at least one uppercase letter, one lowercase letter, and one number.');
      setLoading(false);
      return;
    }

    try {
      const result = await register(email, password, softwareExperience, hardwareExperience, backgroundPreference);
      if (result.success) {
        if (onRegisterSuccess) {
          onRegisterSuccess(result.user, result.profile);
        }
      } else {
        setError(result.error || 'Registration failed. Please try again.');
      }
    } catch (err) {
      console.error('Registration error:', err);
      // Check if the error is due to HTML response instead of JSON
      if (err.message && err.message.includes('Unexpected token') && err.message.includes('<!DOCTYPE')) {
        setError('Server error: Unable to process request. Please check that the backend server is running.');
      } else {
        setError(err.message || 'Registration failed. Please try again.');
      }
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Create Account</h2>
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
            <small className="password-requirements">
              Password must be at least 8 characters with uppercase, lowercase, and number
            </small>
          </div>

          <div className="form-group">
            <label htmlFor="softwareExperience">Software Experience:</label>
            <select
              id="softwareExperience"
              value={softwareExperience}
              onChange={(e) => setSoftwareExperience(e.target.value)}
              disabled={loading}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="hardwareExperience">Hardware Experience:</label>
            <select
              id="hardwareExperience"
              value={hardwareExperience}
              onChange={(e) => setHardwareExperience(e.target.value)}
              disabled={loading}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="backgroundPreference">Background Preference:</label>
            <select
              id="backgroundPreference"
              value={backgroundPreference}
              onChange={(e) => setBackgroundPreference(e.target.value)}
              disabled={loading}
            >
              <option value="software">Software</option>
              <option value="hardware">Hardware</option>
              <option value="mixed">Both</option>
            </select>
          </div>

          <button type="submit" disabled={loading} className="auth-button">
            {loading ? 'Creating Account...' : 'Create Account'}
          </button>
        </form>

        <div className="auth-link">
          <p>Already have an account?{' '}
            <a href="#" onClick={(e) => {
              e.preventDefault();
              if (onSwitchToLogin) {
                onSwitchToLogin();
              }
            }}>
              Login here
            </a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default Register;
export { Register };