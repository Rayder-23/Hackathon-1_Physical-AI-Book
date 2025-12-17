# Data Model: Integrated RAG Chatbot

## Entity Definitions

### ChatSession
- `id` (UUID): Unique identifier for the session
- `user_id` (UUID, nullable): Link to authenticated user
- `created_at` (timestamp): Session creation time
- `updated_at` (timestamp): Last interaction time
- `mode` (enum: 'full-book', 'selected-text'): Query mode for the session
- `metadata` (JSON): Additional session properties

### UserQuery
- `id` (UUID): Unique identifier for the query
- `session_id` (UUID): Reference to ChatSession
- `content` (text): User's question/input
- `created_at` (timestamp): Query timestamp
- `selected_text` (text, nullable): Text selected by user (for selected-text mode)
- `query_embedding` (vector): Embedding of the user query

### RetrievedContent
- `id` (UUID): Unique identifier for retrieved content
- `query_id` (UUID): Reference to UserQuery
- `book_content_id` (UUID): Reference to BookContent
- `relevance_score` (float): Similarity score from vector search
- `retrieved_at` (timestamp): Time of retrieval
- `context_window` (text): Surrounding context of the retrieved content

### AgentResponse
- `id` (UUID): Unique identifier for the response
- `query_id` (UUID): Reference to UserQuery
- `content` (text): LLM-generated response
- `created_at` (timestamp): Response timestamp
- `citations` (JSON): References to BookContent used
- `token_usage` (JSON): Input/output token counts

### BookContent
- `id` (UUID): Unique identifier for book content
- `section_title` (text): Title of the book section
- `content` (text): The actual book content
- `content_embedding` (vector): Embedding of the content
- `page_number` (integer): Page reference in the book
- `chapter` (text): Chapter name/number
- `section` (text): Section within the chapter
- `created_at` (timestamp): When content was indexed
- `updated_at` (timestamp): When content was last updated

## Relationships

```
ChatSession (1) ←→ (N) UserQuery
UserQuery (1) ←→ (N) RetrievedContent
UserQuery (1) ←→ (1) AgentResponse
RetrievedContent (N) ←→ (1) BookContent
```

## Validation Rules

### ChatSession
- Mode must be either 'full-book' or 'selected-text'
- User_id must reference a valid user if provided
- Session must be updated when new queries are added

### UserQuery
- Content must not be empty
- If mode is 'selected-text', selected_text must not be empty
- Query embedding must be generated before storage

### RetrievedContent
- Relevance score must be between 0 and 1
- Must reference valid UserQuery and BookContent
- Context window should be limited to prevent oversized payloads

### AgentResponse
- Content must be non-empty
- Must reference a valid UserQuery
- Citations must reference actual BookContent used

### BookContent
- Content must be non-empty
- Section title must be provided
- Embedding must be generated before storage
- Page number must be positive

## State Transitions

### ChatSession
- ACTIVE (default): Session is active and accepting queries
- INACTIVE: Session has been idle beyond timeout
- ARCHIVED: Session has been cleaned up after retention period

## Indexes

### BookContent
- Vector index on `content_embedding` for similarity search
- Composite index on `chapter` and `section` for content navigation

### UserQuery
- Index on `session_id` and `created_at` for chronological retrieval

### ChatSession
- Index on `user_id` and `updated_at` for user session retrieval
- Index on `created_at` for retention cleanup