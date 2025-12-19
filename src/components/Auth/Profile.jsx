import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';

const Profile = () => {
  const { user, profile, loading, updateProfile, getProfile } = useAuth();
  const [isEditing, setIsEditing] = useState(false);
  const [formData, setFormData] = useState({
    software_experience: '',
    hardware_experience: '',
    background_preference: ''
  });
  const [successMessage, setSuccessMessage] = useState('');
  const [error, setError] = useState('');

  useEffect(() => {
    if (!profile && !loading) {
      getProfile();
    }
    if (profile) {
      setFormData({
        software_experience: profile.software_experience || 'beginner',
        hardware_experience: profile.hardware_experience || 'beginner',
        background_preference: profile.background_preference || 'software'
      });
    }
  }, [profile, loading, getProfile]);

  const handleEdit = () => {
    setIsEditing(true);
    setSuccessMessage('');
    setError('');
  };

  const handleCancel = () => {
    setIsEditing(false);
    if (profile) {
      setFormData({
        software_experience: profile.software_experience || 'beginner',
        hardware_experience: profile.hardware_experience || 'beginner',
        background_preference: profile.background_preference || 'software'
      });
    }
    setSuccessMessage('');
    setError('');
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setSuccessMessage('');

    try {
      const result = await updateProfile(formData);
      if (result.success) {
        setSuccessMessage('Profile updated successfully!');
        setIsEditing(false);
      } else {
        setError(result.error || 'Failed to update profile. Please try again.');
      }
    } catch (err) {
      setError(err.message || 'Failed to update profile. Please try again.');
    }
  };

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  if (loading) {
    return (
      <div className="auth-container">
        <div className="auth-form">
          <h2>Profile</h2>
          <p>Loading profile...</p>
        </div>
      </div>
    );
  }

  if (!user) {
    return (
      <div className="auth-container">
        <div className="auth-form">
          <h2>Profile</h2>
          <div className="error-message">
            Please log in to view your profile.
            <br />
            <a href="/Hackathon-1_Physical-AI-Book/login" style={{ color: 'inherit', textDecoration: 'underline' }}>Go to Login</a>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-container">
      <div className="auth-form">
        <h2>Your Profile</h2>

        {error && <div className="error-message">{error}</div>}
        {successMessage && <div className="success-message">{successMessage}</div>}

        {isEditing ? (
          <form onSubmit={handleSubmit}>
            <div className="form-group">
              <label htmlFor="email">Email:</label>
              <p>{user?.email || 'Not available'}</p>
            </div>

            <div className="form-group">
              <label htmlFor="software_experience">Software Experience:</label>
              <select
                id="software_experience"
                name="software_experience"
                value={formData.software_experience}
                onChange={handleChange}
                required
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
                required
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
                required
              >
                <option value="software">Software</option>
                <option value="hardware">Hardware</option>
                <option value="mixed">Both</option>
              </select>
            </div>

            <button type="submit" className="auth-button" style={{ marginRight: '10px' }}>
              Save Changes
            </button>
            <button type="button" className="auth-button-secondary" onClick={handleCancel}>
              Cancel
            </button>
          </form>
        ) : (
          <div>
            <div className="form-group">
              <label>Email:</label>
              <p>{user?.email || 'Not available'}</p>
            </div>

            <div className="form-group">
              <label>Software Experience:</label>
              <p>{profile.software_experience || 'Not specified'}</p>
            </div>

            <div className="form-group">
              <label>Hardware Experience:</label>
              <p>{profile.hardware_experience || 'Not specified'}</p>
            </div>

            <div className="form-group">
              <label>Background Preference:</label>
              <p>{profile.background_preference || 'Not specified'}</p>
            </div>

            <button onClick={handleEdit} className="auth-button">
              Edit Profile
            </button>
          </div>
        )}
      </div>
    </div>
  );
};

export default Profile;
export { Profile };