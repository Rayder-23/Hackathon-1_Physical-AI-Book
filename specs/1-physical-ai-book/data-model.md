# Data Model: Physical AI & Humanoid Robotics Book

## User Data Model

### User Profile
- **id**: string (unique identifier)
- **email**: string (user's email address, validated format)
- **name**: string (user's full name)
- **role**: enum ['student', 'educator', 'practitioner'] (user type classification)
- **background**: object (software and hardware experience details)
  - **software**: string (programming experience level and languages)
  - **hardware**: string (robotics and embedded systems experience)
  - **domain**: string (specific area of expertise or interest)
- **createdAt**: datetime (account creation timestamp)
- **lastAccessed**: datetime (last login timestamp)

### Validation Rules
- Email must be in valid format
- Role must be one of the specified enum values
- Background fields are required at signup
- User ID must be unique across system

## Content Data Model

### Book Module
- **id**: string (unique module identifier, e.g., "module-1-ros2")
- **title**: string (module title)
- **description**: string (brief module description)
- **order**: integer (sequence in book progression)
- **sections**: array of Section objects (content sections within module)
- **learningOutcomes**: array of strings (what user should understand)
- **prerequisites**: array of strings (required knowledge areas)
- **estimatedTime**: integer (minutes to complete module)

### Section
- **id**: string (unique section identifier)
- **title**: string (section title)
- **content**: string (main content in MDX format)
- **order**: integer (sequence within module)
- **type**: enum ['text', 'example', 'diagram', 'exercise', 'reference']
- **relatedTopics**: array of strings (linked concepts in other sections)

### Content Reference
- **id**: string (unique reference identifier)
- **title**: string (reference title)
- **author**: string (author name(s))
- **year**: integer (publication year)
- **source**: string (publication venue, URL, or other source info)
- **type**: enum ['book', 'paper', 'documentation', 'website', 'video']
- **accessedDate**: datetime (when reference was accessed for citation)

## Interaction Data Model

### User Progress
- **userId**: string (reference to User Profile)
- **moduleId**: string (reference to Book Module)
- **completedSections**: array of strings (IDs of completed sections)
- **overallProgress**: float (0-100 percentage)
- **lastSection**: string (ID of last accessed section)
- **timeSpent**: integer (minutes spent on module)
- **completedAt**: datetime (when module was completed, null if in progress)

### Content Interaction
- **id**: string (unique interaction identifier)
- **userId**: string (reference to User Profile)
- **contentId**: string (ID of content interacted with)
- **interactionType**: enum ['view', 'complete', 'bookmark', 'note', 'quiz']
- **timestamp**: datetime (when interaction occurred)
- **metadata**: object (additional data specific to interaction type)

## Validation Rules

### User Profile Validation
- Email format must follow RFC 5322 standard
- Role must be one of the three specified values
- Background fields must be provided at account creation
- User ID must be unique across the system

### Content Model Validation
- Module IDs must be unique within the book
- Module order numbers must be sequential without gaps
- Section types must be one of the specified enum values
- Learning outcomes must be provided for each module
- Estimated time must be a positive integer

### Progress Tracking Validation
- User progress records must reference valid users
- Module references in progress must be valid
- Progress percentage must be between 0 and 100
- Time spent must be non-negative

## Relationships

### User to Content
- One User can have progress records for many Modules
- One Module can have progress records for many Users

### Module to Section
- One Module contains many Sections
- One Section belongs to one Module

### Content to Reference
- One Section can reference many Content References
- One Content Reference can be referenced by many Sections