import React, { useState } from 'react';
import { getUserBackground, updateUserProfile } from '../../config/auth';

const SignupQuestionnaire = ({ onComplete }) => {
  const [formData, setFormData] = useState({
    softwareBackground: '',
    hardwareBackground: '',
    experienceLevel: 'intermediate'
  });

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    // Save user background information
    const profileData = {
      softwareBackground: formData.softwareBackground,
      hardwareBackground: formData.hardwareBackground,
      experienceLevel: formData.experienceLevel,
      completedQuestionnaire: true,
      questionnaireCompletedAt: new Date().toISOString()
    };

    // Update user profile in localStorage
    const success = updateUserProfile(profileData);

    if (success && onComplete) {
      onComplete(profileData);
    }
  };

  return (
    <div className="signup-questionnaire">
      <h2>Tell Us About Your Background</h2>
      <p>This information will help us personalize your learning experience in Physical AI and Robotics.</p>

      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="softwareBackground">
            <strong>Software Background:</strong>
          </label>
          <textarea
            id="softwareBackground"
            name="softwareBackground"
            value={formData.softwareBackground}
            onChange={handleChange}
            placeholder="Describe your software development experience, programming languages you know, and any robotics software experience..."
            rows="4"
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="hardwareBackground">
            <strong>Hardware Background:</strong>
          </label>
          <textarea
            id="hardwareBackground"
            name="hardwareBackground"
            value={formData.hardwareBackground}
            onChange={handleChange}
            placeholder="Describe your hardware experience, electronics knowledge, and any robotics hardware experience..."
            rows="4"
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="experienceLevel">
            <strong>Overall Experience Level:</strong>
          </label>
          <select
            id="experienceLevel"
            name="experienceLevel"
            value={formData.experienceLevel}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <button type="submit" className="button button--primary">
          Complete Profile
        </button>
      </form>
    </div>
  );
};

export default SignupQuestionnaire;
export { SignupQuestionnaire };