# Quickstart Guide: Authentication and User Profiling

## Overview
This guide provides a quick setup for the authentication and user profiling system using better-auth with Neon Postgres and FastAPI backend integration.

## Prerequisites
- Node.js 18+ for frontend development
- Python 3.11+ for backend development
- Neon Postgres database instance
- Better-auth library (v0.0.25+)
- FastAPI (v0.104+)

## Environment Setup

### 1. Database Configuration
```bash
# Set up Neon Postgres connection
export DATABASE_URL="postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require"
```

### 2. Better-Auth Configuration
```javascript
// Create auth config file
import { NextAuth } from 'next-auth'
import { neon } from '@neondatabase/serverless'
import { BetterAuthAdapter } from '@better-auth/neon-adapter'

const db = neon(process.env.DATABASE_URL)

export const auth = NextAuth({
  adapter: BetterAuthAdapter(db),
  secret: process.env.AUTH_SECRET,
  database: {
    provider: 'postgresql',
    url: process.env.DATABASE_URL,
  },
  // Additional auth configuration
})
```

### 3. FastAPI Backend Integration
```python
# Add auth endpoints to existing FastAPI app
from fastapi import FastAPI, Depends
from backend.auth.services import get_current_user
from backend.auth.models import UserProfile

app = FastAPI()

@app.post("/api/auth/profile")
async def update_profile(profile_data: UserProfile, current_user = Depends(get_current_user)):
    # Update user profile logic
    pass
```

## Frontend Integration

### 1. Authentication Components
```jsx
// src/components/Auth/Login.jsx
import { useAuth } from 'better-auth/react'

export function Login() {
  const { signIn } = useAuth()

  const handleLogin = async (email, password) => {
    const result = await signIn.email({
      email,
      password,
      callbackURL: "/dashboard"
    })
    return result
  }

  return (
    // Login form JSX
  )
}
```

### 2. Profile Collection During Registration
```jsx
// src/components/Auth/Register.jsx
import { useAuth } from 'better-auth/react'

export function Register() {
  const { signUp } = useAuth()

  const handleRegistration = async (userData) => {
    // Include profile data in registration
    const result = await signUp.email({
      email: userData.email,
      password: userData.password,
      name: userData.name,
      profileData: {
        software_experience: userData.software_experience,
        hardware_experience: userData.hardware_experience,
        background_preference: userData.background_preference
      }
    })
    return result
  }

  return (
    // Registration form with profile fields
  )
}
```

## Database Schema Setup

Run the following SQL to create the required tables:

```sql
-- Users table is managed by better-auth
-- Profile table for user background information
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    software_experience VARCHAR(20) CHECK (software_experience IN ('beginner', 'intermediate', 'advanced')),
    hardware_experience VARCHAR(20) CHECK (hardware_experience IN ('beginner', 'intermediate', 'advanced')),
    background_preference VARCHAR(20) CHECK (background_preference IN ('software', 'hardware', 'mixed')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
```

## Testing the Setup

### 1. Unit Tests
```bash
# Backend tests
python -m pytest backend/auth/tests/

# Frontend tests
npm test src/components/Auth/
```

### 2. Integration Tests
```bash
# Test registration flow
curl -X POST http://localhost:8000/api/auth/register \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "securePassword123",
    "software_experience": "intermediate",
    "hardware_experience": "beginner",
    "background_preference": "mixed"
  }'
```

## Next Steps
1. Implement the authentication context for Docusaurus integration
2. Create profile management UI components
3. Integrate with existing RAG system for personalization
4. Add email verification functionality
5. Implement session management for static hosting compatibility