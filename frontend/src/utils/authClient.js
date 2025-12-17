// Better-Auth API Client for Docusaurus (static hosting compatible)
import { createAuthClient } from "better-auth/client";

// Initialize Better-Auth client with proper configuration
// This requires the same configuration that was used on the server side
const authClient = createAuthClient({
  // This configuration should match your server-side better-auth configuration
  // For static hosting, we assume the auth endpoints are available at the origin
});

export default authClient;