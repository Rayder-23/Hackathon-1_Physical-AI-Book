# Data Model: Chatkit-Based RAG Chatbot Integration

## Entity Definitions

### User Session
- **id**: string (UUID) - Unique identifier for the session
- **userId**: string - Reference to authenticated user (null for guest sessions)
- **sessionId**: string - Chatkit session identifier
- **createdAt**: datetime - Timestamp when session was created
- **lastActivityAt**: datetime - Timestamp of last activity
- **expiresAt**: datetime - Expiration time (for guest sessions)
- **isActive**: boolean - Whether the session is currently active

### Guest Session
- **id**: string (UUID) - Unique identifier for the guest session
- **sessionId**: string - Temporary session identifier
- **createdAt**: datetime - Timestamp when session was created
- **lastActivityAt**: datetime - Timestamp of last activity
- **expiresAt**: datetime - Expiration time (24 hours after last activity)
- **isActive**: boolean - Whether the session is currently active

### Chat Message
- **id**: string (UUID) - Unique identifier for the message
- **roomId**: string - Chatkit room identifier
- **senderId**: string - User or system identifier
- **senderType**: enum('user', 'bot', 'system') - Type of message sender
- **content**: string - The message content
- **timestamp**: datetime - When the message was sent
- **messageType**: enum('text', 'command', 'response') - Type of message
- **relatedQueryId**: string (optional) - Reference to related query in RAG system

### Conversation Thread
- **id**: string (UUID) - Unique identifier for the conversation
- **roomId**: string - Chatkit room identifier
- **sessionId**: string - Reference to user or guest session
- **title**: string - Conversation title (auto-generated from first query)
- **createdAt**: datetime - When conversation was started
- **lastMessageAt**: datetime - Timestamp of last message
- **messageCount**: integer - Number of messages in conversation
- **isActive**: boolean - Whether conversation is currently active

### RAG Query
- **id**: string (UUID) - Unique identifier for the query
- **sessionId**: string - Reference to user session
- **queryText**: string - Original user query
- **queryVector**: string - Vector representation of query (for Qdrant)
- **contextDocuments**: array - Relevant documents retrieved from vector store
- **responseText**: string - Generated response to the query
- **timestamp**: datetime - When query was processed
- **responseTimeMs**: integer - Time taken to process the query
- **relevanceScore**: float - Score of how relevant the response was

## Database Schema (Neon Postgres)

### sessions table
```sql
CREATE TABLE sessions (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id VARCHAR(255),
  chatkit_session_id VARCHAR(255) NOT NULL,
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  last_activity_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  expires_at TIMESTAMP WITH TIME ZONE,
  is_active BOOLEAN DEFAULT TRUE
);

CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_chatkit_session_id ON sessions(chatkit_session_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
```

### conversations table
```sql
CREATE TABLE conversations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chatkit_room_id VARCHAR(255) NOT NULL,
  session_id UUID REFERENCES sessions(id),
  title VARCHAR(500),
  created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  last_message_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  message_count INTEGER DEFAULT 0,
  is_active BOOLEAN DEFAULT TRUE
);

CREATE INDEX idx_conversations_session_id ON conversations(session_id);
CREATE INDEX idx_conversations_chatkit_room_id ON conversations(chatkit_room_id);
```

### rag_queries table
```sql
CREATE TABLE rag_queries (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  session_id UUID REFERENCES sessions(id),
  query_text TEXT NOT NULL,
  context_documents JSONB,
  response_text TEXT,
  timestamp TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
  response_time_ms INTEGER,
  relevance_score FLOAT
);

CREATE INDEX idx_rag_queries_session_id ON rag_queries(session_id);
CREATE INDEX idx_rag_queries_timestamp ON rag_queries(timestamp);
```

## Validation Rules

### User Session
- Must have either a userId (authenticated) or be marked as guest session
- expiresAt must be in the future for active sessions
- createdAt must be before lastActivityAt

### Chat Message
- content must not be empty
- timestamp must be current or past
- senderId must reference a valid user or be system identifier

### Conversation Thread
- title must be auto-generated if not provided
- messageCount must match actual number of messages in the thread
- isActive reflects whether the conversation is ongoing

### RAG Query
- queryText must be non-empty
- contextDocuments must be valid JSON
- responseTimeMs must be positive
- relevanceScore must be between 0 and 1

## State Transitions

### Session States
- **ACTIVE**: Session is created and user is interacting
- **INACTIVE**: No activity for a period but still valid
- **EXPIRED**: Session has exceeded time limit
- **TERMINATED**: Session explicitly ended by user or system

### Conversation States
- **OPEN**: Conversation is active and accepting new messages
- **PAUSED**: Conversation inactive but preserved
- **CLOSED**: Conversation completed and read-only
- **ARCHIVED**: Conversation moved to long-term storage

## Relationships

- User Session (1) → (0..n) Conversation Thread
- Conversation Thread (1) → (0..n) Chat Message
- User Session (1) → (0..n) RAG Query
- Chat Message (1) → (0..1) RAG Query (when message is a query response)