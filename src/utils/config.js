// Configuration utility for Docusaurus
// This gets the backend URL from environment variables during build time

// Default backend URL - this will be overridden by environment variable if available
// LOCAL BACKEND (current configuration):
// const BACKEND_URL =
//   (typeof process !== 'undefined' && process.env && process.env.REACT_APP_BACKEND_URL)
//     ? process.env.REACT_APP_BACKEND_URL
//     : 'http://localhost:8005';

// RAILWAY BACKEND (for testing with deployed backend):
const BACKEND_URL = 'https://web-production-6a11c.up.railway.app';

// Export the configuration
export const config = {
  BACKEND_URL: BACKEND_URL
};

// For debugging purposes, log the backend URL in development
if (typeof window !== 'undefined' &&
    typeof process !== 'undefined' &&
    process.env &&
    process.env.NODE_ENV === 'development') {
  console.log('Backend URL configured as:', BACKEND_URL);
}

export default config;