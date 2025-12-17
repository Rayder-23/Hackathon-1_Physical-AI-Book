# API Endpoints Documentation

## Overview
This document describes the API endpoints for the Physical AI Book RAG Chatbot.

## Base URL
All endpoints are relative to `/api`

## Authentication
No authentication required for basic functionality. Session management is handled via session IDs.

## Endpoints

### POST /api/chat
Process a chat message and return an AI-generated response.

#### Parameters
- `message` (string, required): The user's message/question
- `session_id` (string, optional): Existing session ID to continue conversation
- `mode` (string, optional): Query mode - "full-book" (default) or "selected-text"
- `selected_text` (string, optional): Text selected by user (required for selected-text mode)

#### Example Request
```json
{
  "message": "What are the fundamental principles of Physical AI?",
  "mode": "full-book"
}
```

#### Example Response
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "session_id": "550e8400-e29b-41d4-a716-446655440001",
  "message": "The fundamental principles of Physical AI include perception, reasoning, and action...",
  "citations": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440002",
      "chapter": "Chapter 1",
      "section": "Introduction",
      "page": 1,
      "relevance_score": 0.85
    }
  ],
  "timestamp": "2023-10-20T10:30:00Z"
}
```

#### Error Responses
- `400 Bad Request`: Invalid mode or missing selected_text for selected-text mode
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Processing error

### POST /api/retrieve
Perform content retrieval from the book without generating a response.

#### Parameters
- `query` (string, required): The search query
- `mode` (string, optional): Query mode - "full-book" (default) or "selected-text"
- `selected_text` (string, optional): Text selected by user (for selected-text mode)
- `limit` (integer, optional): Maximum number of results (default: 5)

#### Example Request
```json
{
  "query": "sensor fusion in robotics",
  "mode": "full-book",
  "limit": 3
}
```

#### Example Response
```json
{
  "results": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "content": "Sensor fusion is the process of combining sensory data...",
      "metadata": {
        "section_title": "Perception in Physical AI",
        "chapter": "Chapter 2",
        "section": "Sensory Integration",
        "page_number": 15
      },
      "relevance_score": 0.92
    }
  ]
}
```

### GET /api/sessions/{session_id}
Get details about a specific chat session.

#### Path Parameters
- `session_id` (string): The session ID to retrieve

#### Example Response
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": "550e8400-e29b-41d4-a716-446655440001",
  "mode": "full-book",
  "created_at": "2023-10-20T10:00:00Z",
  "updated_at": "2023-10-20T10:30:00Z"
}
```

### GET /health
Health check endpoint to verify API availability.

#### Example Response
```json
{
  "status": "healthy",
  "timestamp": "2023-10-20T10:30:00Z"
}
```

## Rate Limiting
The API implements rate limiting to prevent abuse:
- Default: 100 requests per hour per IP address
- Configurable via environment variables
- Returns HTTP 429 when limit is exceeded

## Query Modes
- **full-book**: Searches the entire book content for relevant information
- **selected-text**: Restricts search to content related to user-selected text only

## Error Handling
All error responses follow this format:
```json
{
  "detail": "Error message describing the issue"
}
```