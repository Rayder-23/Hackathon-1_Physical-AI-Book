# Data Model: Physical AI Book (Iteration 2)

**Date**: 2025-12-08
**Feature**: Physical AI Book (Iteration 2)

## Entity Definitions

### User Profile
**Description**: Represents a registered user with software/hardware background information and preferences for personalization
**Fields**:
- id: Unique identifier for the user
- email: User's email address (for authentication)
- name: User's display name
- softwareBackground: User's software experience level and areas of expertise
- hardwareBackground: User's hardware experience level and areas of expertise
- preferences: Personalization settings and preferences
- createdAt: Account creation timestamp
- lastLogin: Last login timestamp

**Validation Rules**:
- Email must be valid format
- Software and hardware background must be non-empty strings
- Preferences must be a valid JSON object

### Book Module
**Description**: Represents one of the four core modules (ROS 2, Simulation, Isaac AI, VLA) with associated content
**Fields**:
- id: Unique identifier for the module
- title: Display title of the module
- slug: URL-friendly identifier
- description: Brief description of the module
- order: Position in the book sequence
- chapters: Array of chapter IDs belonging to this module

**Validation Rules**:
- Title and slug must be non-empty
- Order must be a positive integer
- Chapters must be valid chapter references

### Chapter Content
**Description**: Represents individual chapters that can be personalized based on user profile
**Fields**:
- id: Unique identifier for the chapter
- moduleId: Reference to the parent module
- title: Display title of the chapter
- slug: URL-friendly identifier
- content: The main content of the chapter
- personalizedContent: Alternative content for personalized experience
- urduContent: Urdu translation of the chapter
- order: Position within the module
- metadata: Additional information about the chapter

**Validation Rules**:
- Title and slug must be non-empty
- Order must be a positive integer
- Content must be valid MDX format

### Translation Pair
**Description**: Represents English and Urdu MDX content that can be swapped based on user preference
**Fields**:
- id: Unique identifier for the translation pair
- englishContentId: Reference to the English chapter content
- urduContent: The Urdu translation of the content
- isTranslated: Boolean indicating if translation is complete
- lastUpdated: Timestamp of last translation update

**Validation Rules**:
- English content ID must reference a valid chapter
- Urdu content must be valid MDX format if provided
- isTranslated must be boolean

### Access Level
**Description**: Represents user permissions (public, author, admin) that determine content access
**Fields**:
- id: Unique identifier for the access level
- name: Name of the access level (public, author, admin)
- permissions: List of permissions associated with this level
- description: Description of what this access level allows

**Validation Rules**:
- Name must be one of the predefined values (public, author, admin)
- Permissions must be valid permission strings

## Relationships

```
User Profile 1 -- * Access Level
User Profile 1 -- 1 User Access Level (current user's level)

Book Module 1 -- * Chapter Content
Chapter Content 1 -- 1 Translation Pair (optional)

User Profile * -- * Chapter Content (through personalization preferences)
```

## State Transitions

### User Profile States
- **Unregistered**: User has not signed up
- **Registered**: User has completed signup with basic information
- **Profile Complete**: User has provided complete background information
- **Authenticated**: User is currently logged in

### Chapter Content States
- **Draft**: Content is being created
- **Published**: Content is available to users
- **Personalized**: Content has personalization options
- **Translated**: Content has Urdu translation available

### Translation Pair States
- **Not Started**: No translation has been created
- **In Progress**: Translation is being worked on
- **Complete**: Translation is finished and validated
- **Validated**: Translation has passed quality checks

## Validation Rules Summary

1. **User Profile**:
   - Email uniqueness across all profiles
   - Required background information for personalization
   - Valid JSON format for preferences

2. **Book Module**:
   - Unique slugs within the book
   - Sequential ordering without gaps
   - Non-empty content requirement

3. **Chapter Content**:
   - Valid MDX syntax
   - Unique slugs within the module
   - Proper parent-child relationship with modules

4. **Translation Pair**:
   - Valid reference to existing English content
   - Valid MDX syntax for Urdu content
   - Consistent structure between translations

5. **Access Level**:
   - Predefined permission sets
   - Hierarchical permission inheritance (admin > author > public)