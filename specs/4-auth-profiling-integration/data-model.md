# Data Model: Authentication and User Profiling

## Entities

### User
Represents a registered user account with authentication credentials

**Fields**:
- `id` (UUID): Unique identifier for the user
- `email` (string): User's email address (unique, validated)
- `password_hash` (string): Securely hashed password
- `created_at` (timestamp): Account creation timestamp
- `updated_at` (timestamp): Last update timestamp
- `is_active` (boolean): Account active status
- `email_verified` (boolean): Email verification status

**Validation rules**:
- Email must be valid format and unique
- Password must meet strength requirements
- Email must be unique across all users

**Relationships**:
- One-to-One: UserProfile (user_id → user.id)

### UserProfile
Contains user background information for content personalization

**Fields**:
- `id` (UUID): Unique identifier for the profile
- `user_id` (UUID): Foreign key to User table
- `software_experience` (string): User's software background level (beginner, intermediate, advanced)
- `hardware_experience` (string): User's hardware background level (beginner, intermediate, advanced)
- `background_preference` (string): Primary background focus (software, hardware, mixed)
- `created_at` (timestamp): Profile creation timestamp
- `updated_at` (timestamp): Last profile update timestamp

**Validation rules**:
- user_id must reference an existing User
- software_experience must be one of: 'beginner', 'intermediate', 'advanced'
- hardware_experience must be one of: 'beginner', 'intermediate', 'advanced'
- background_preference must be one of: 'software', 'hardware', 'mixed'

**Relationships**:
- Many-to-One: User (user_id → user.id)

## Database Schema

```sql
-- Users table (managed by better-auth)
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    is_active BOOLEAN DEFAULT TRUE,
    email_verified BOOLEAN DEFAULT FALSE
);

-- User profiles table (custom for this feature)
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    software_experience VARCHAR(20) CHECK (software_experience IN ('beginner', 'intermediate', 'advanced')),
    hardware_experience VARCHAR(20) CHECK (hardware_experience IN ('beginner', 'intermediate', 'advanced')),
    background_preference VARCHAR(20) CHECK (background_preference IN ('software', 'hardware', 'mixed')),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_users_email ON users(email);
```

## State Transitions

### User Account States
- `unregistered` → `registered` (via registration flow)
- `registered` → `email_verified` (via email verification)
- `active` ↔ `inactive` (via admin action or user choice)

### Profile States
- `incomplete` → `complete` (when profile data is first set)
- `complete` → `updated` (when profile data is modified)

## Access Patterns

### Authentication
- Find user by email for login
- Validate password hash during authentication
- Check user active status during session validation

### Profile Management
- Get profile by user_id for personalization
- Update profile data during registration/edit
- Query profiles for personalization features

### Security Considerations
- Password hashes stored using bcrypt or similar
- Email verification required for full access
- Profile data access limited to authenticated users
- Foreign key constraints ensure data integrity