import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';

const SignupQuestionnaire = ({ onComplete }) => {
  const [formData, setFormData] = useState({
    software_experience: 'beginner',
    hardware_experience: 'beginner',
    background_preference: 'software'
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { updateProfile, profile } = useAuth();

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const result = await updateProfile(formData);
      if (result.success) {
        if (onComplete) {
          onComplete(result.profile);
        }
      } else {
        setError(result.error || 'Failed to update profile. Please try again.');
      }
    } catch (err) {
      setError(err.message || 'Failed to update profile. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Tell Us About Your Background</h2>
        <p>This information will help us personalize your learning experience in Physical AI and Robotics.</p>

        {error && <div className="error-message">{error}</div>}

        <form onSubmit={handleSubmit}>
          <div className="form-group">
            <label htmlFor="software_experience">Software Experience:</label>
            <select
              id="software_experience"
              name="software_experience"
              value={formData.software_experience}
              onChange={handleChange}
              disabled={loading}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="hardware_experience">Hardware Experience:</label>
            <select
              id="hardware_experience"
              name="hardware_experience"
              value={formData.hardware_experience}
              onChange={handleChange}
              disabled={loading}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className="form-group">
            <label htmlFor="background_preference">Background Preference:</label>
            <select
              id="background_preference"
              name="background_preference"
              value={formData.background_preference}
              onChange={handleChange}
              disabled={loading}
            >
              <option value="software">Software</option>
              <option value="hardware">Hardware</option>
              <option value="both">Both</option>
            </select>
          </div>

          <button type="submit" disabled={loading} className="auth-button">
            {loading ? 'Saving...' : 'Complete Profile'}
          </button>
        </form>
      </div>
    </div>
  );
};

export default SignupQuestionnaire;
export { SignupQuestionnaire };