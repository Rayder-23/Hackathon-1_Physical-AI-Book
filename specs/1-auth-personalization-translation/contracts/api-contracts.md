# API Contracts: Authentication, Personalization, and Translation Features

## Authentication Endpoints (Client-Only)

### POST /api/auth/login
**Description**: Authenticate user with username and password
**Request**:
```json
{
  "username": "string (required)",
  "password": "string (required)"
}
```
**Response**:
```json
{
  "success": "boolean",
  "user": {
    "id": "string",
    "username": "string",
    "email": "string (optional)"
  },
  "token": "string"
}
```
**Error Response**:
```json
{
  "error": "string",
  "message": "string"
}
```

### POST /api/auth/register
**Description**: Register new user account
**Request**:
```json
{
  "username": "string (required, 3-30 chars)",
  "email": "string (optional)",
  "password": "string (required, min 8 chars)"
}
```
**Response**:
```json
{
  "success": "boolean",
  "user": {
    "id": "string",
    "username": "string",
    "email": "string (optional)"
  }
}
```

### POST /api/auth/logout
**Description**: Log out current user
**Response**:
```json
{
  "success": "boolean"
}
```

## Personalization Endpoints (Client-Only)

### GET /api/personalization/settings
**Description**: Get current user's personalization settings
**Response**:
```json
{
  "difficulty": "string (beginner|intermediate|advanced)",
  "focus": "string (software|hardware|mixed)"
}
```

### PUT /api/personalization/settings
**Description**: Update user's personalization settings
**Request**:
```json
{
  "difficulty": "string (optional)",
  "focus": "string (optional)"
}
```
**Response**:
```json
{
  "success": "boolean",
  "settings": {
    "difficulty": "string",
    "focus": "string"
  }
}
```

## Translation Endpoints (Client-Only)

### GET /api/translation/available
**Description**: Get list of available languages
**Response**:
```json
{
  "languages": [
    {
      "code": "string (en, ur)",
      "name": "string",
      "isAvailable": "boolean"
    }
  ]
}
```

### GET /api/translation/{lang}/{page}
**Description**: Get translations for specific page in target language
**Response**:
```json
{
  "page": "string",
  "language": "string",
  "translations": {
    "key": "translation text"
  }
}
```

## Client-Side State Management Contracts

### Authentication State
```javascript
{
  user: {
    id: string,
    username: string,
    email: string,
    isAuthenticated: boolean
  },
  loading: boolean,
  error: string
}
```

### Personalization State
```javascript
{
  settings: {
    difficulty: 'beginner' | 'intermediate' | 'advanced',
    focus: 'software' | 'hardware' | 'mixed'
  }
}
```

### Translation State
```javascript
{
  currentLanguage: 'en' | 'ur',
  availableTranslations: object,
  loading: boolean
}
```

## Component Interface Contracts

### useAuth Hook
```javascript
const { user, login, logout, register, loading } = useAuth();
```

### usePersonalization Hook
```javascript
const { settings, updateSettings } = usePersonalization();
```

### useTranslation Hook
```javascript
const { language, toggleLanguage, translations } = useTranslation();
```

## Error Handling Contracts

### Standard Error Format
```javascript
{
  code: string,
  message: string,
  details?: object
}
```

### Expected Error Codes
- `AUTH_001`: Invalid credentials
- `AUTH_002`: User not found
- `AUTH_003`: Session expired
- `PER_001`: Invalid personalization settings
- `TRANS_001`: Translation not available
- `SSR_001`: Operation not allowed during SSR