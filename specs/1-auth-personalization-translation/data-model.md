# Data Model: Authentication, Personalization, and Translation Features

## User Entity
- **id**: Unique identifier for the user
- **username**: User's chosen username (required, unique)
- **email**: User's email address (optional)
- **password**: Hashed password (stored securely)
- **createdAt**: Account creation timestamp
- **lastLoginAt**: Last login timestamp
- **profile**: Object containing user preferences

## Profile Entity
- **userId**: Reference to the associated user
- **personalization**: Object containing personalization settings
  - **difficulty**: User's preferred content difficulty (beginner, intermediate, advanced)
  - **focus**: User's content focus preference (software, hardware, mixed)
  - **language**: User's preferred language (English, Urdu)

## Session Entity
- **userId**: Reference to the authenticated user
- **token**: Authentication token (stored in localStorage)
- **expiresAt**: Token expiration timestamp
- **createdAt**: Session creation timestamp

## Translation Entity
- **key**: Unique identifier for the translatable text segment
- **en**: English content
- **ur**: Urdu content
- **context**: Description of where this text is used
- **category**: Content category (navigation, content, UI, etc.)

## Personalization Settings Entity
- **userId**: Reference to the associated user
- **difficulty**: Content difficulty preference (beginner, intermediate, advanced)
- **focus**: Content focus preference (software, hardware, mixed)
- **createdAt**: Settings creation timestamp
- **updatedAt**: Settings last updated timestamp

## Validation Rules
- Username must be 3-30 characters, alphanumeric with underscores allowed
- Email must follow standard email format if provided
- Password must be at least 8 characters with mixed case and numbers
- Personalization settings require authentication
- Translation keys must be unique within their category

## State Transitions
- User registration: Unauthenticated → Pending verification → Active
- User login: Unauthenticated → Authenticated
- User logout: Authenticated → Unauthenticated
- Personalization toggle: Default → Personalized
- Translation toggle: English → Urdu (or vice versa)