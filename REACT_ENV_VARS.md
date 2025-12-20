# REACT_APP_BACKEND_URL Configuration

This document explains how the `REACT_APP_BACKEND_URL` environment variable is used and how to configure it for different environments.

## What is REACT_APP_BACKEND_URL?

`REACT_APP_BACKEND_URL` is a React environment variable that specifies the base URL for all backend API calls in the frontend application. It replaces hardcoded URLs like `http://localhost:8005` with a configurable value.

## How It Works

In the code, API calls now follow this pattern:
```javascript
const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8005';
const response = await fetch(`${backendUrl}/api/chat`, { ... });
```

This means:
- If `REACT_APP_BACKEND_URL` is set, it will use that value
- If not set, it will fall back to `http://localhost:8005`

## Where It's Stored

### 1. Local Development (.env file)
Create a `.env` file in the project root directory:
```
REACT_APP_BACKEND_URL=http://localhost:8005
```

### 2. Build Time Environment
When building the React app, environment variables are embedded at build time.

### 3. Deployment Platforms

#### For Railway Deployment:
- Set the environment variable in Railway dashboard under your frontend service
- Variable name: `REACT_APP_BACKEND_URL`
- Variable value: Your backend URL (e.g., `https://your-backend-app-production.up.railway.app`)

#### For Netlify/Vercel/Other Platforms:
- Set in deployment platform's environment variables section
- Variable name: `REACT_APP_BACKEND_URL`
- Variable value: Your deployed backend URL

## Configuration Examples

### Local Development
Create `.env` file in project root:
```
REACT_APP_BACKEND_URL=http://localhost:8005
```

### Production (Railway)
In Railway dashboard:
```
Variable: REACT_APP_BACKEND_URL
Value: https://your-app-production.up.railway.app
```

### Production (Vercel)
In Vercel dashboard under Environment Variables:
```
Key: REACT_APP_BACKEND_URL
Value: https://your-app.vercel.app
```

## Files That Use This Variable

The following frontend files have been updated to use this environment variable:
- `src/components/Chatkit/ChatkitWidget.jsx`
- `src/components/Chatkit/ChatInterface.jsx`
- `src/components/Auth/auth-nav.js`
- `src/components/Auth/NavbarAuth.jsx`
- `src/contexts/AuthContext.jsx`

## Important Notes

1. **React Environment Variable Convention**: React only recognizes environment variables that start with `REACT_APP_`

2. **Build Time vs Runtime**: Environment variables are embedded at build time, not runtime. You need to rebuild the application when changing these values.

3. **Local Development**: Create a `.env` file in the project root for local development.

4. **Security**: Never commit `.env` files to version control. Add them to `.gitignore`.

5. **Fallback**: If the environment variable is not set, the application will fall back to `http://localhost:8005`.

## Setting Up for Different Environments

### Development Environment
1. Create `.env` in project root
2. Add: `REACT_APP_BACKEND_URL=http://localhost:8005`
3. Start development server: `npm start`

### Production Deployment
1. Set environment variable in your deployment platform
2. Build the application: `npm run build`
3. Deploy the built files

## Troubleshooting

If API calls are failing:
1. Check that the environment variable is set correctly
2. Verify the backend URL is accessible
3. Ensure the URL includes the protocol (http:// or https://)
4. Check browser developer tools for CORS errors
5. Confirm the backend service is running and accessible

## Example .env File
```
# Backend API URL
REACT_APP_BACKEND_URL=http://localhost:8005

# Other environment variables can be added here as needed
# REACT_APP_API_KEY=your_api_key_here
```

Remember to add `.env` to your `.gitignore` file to keep sensitive information secure.