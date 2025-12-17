/**
 * @typedef {Object} UserProfile
 * @property {string} id - Unique identifier for the user
 * @property {string} email - User's email address (for authentication)
 * @property {string} name - User's display name
 * @property {string} softwareBackground - User's software experience level and areas of expertise
 * @property {string} hardwareBackground - User's hardware experience level and areas of expertise
 * @property {Object} preferences - Personalization settings and preferences
 * @property {string} createdAt - Account creation timestamp
 * @property {string} lastLogin - Last login timestamp
 */

/**
 * @typedef {Object} UserBackground
 * @property {string} softwareBackground - User's software background and experience
 * @property {string} hardwareBackground - User's hardware background and experience
 */

/**
 * @typedef {Object} PersonalizationSettings
 * @property {boolean} enabled - Whether personalization is enabled
 * @property {'beginner'|'intermediate'|'advanced'} difficulty - Difficulty level preference
 * @property {'software'|'hardware'|'mixed'} background - Background preference
 * @property {Object} preferences - Additional personalization preferences
 */

export {}; // Empty export to make this file a module