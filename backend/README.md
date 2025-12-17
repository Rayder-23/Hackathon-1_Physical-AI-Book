# Physical AI Book RAG Chatbot Backend

This is the backend service for the Physical AI Book RAG Chatbot, built with FastAPI and using OpenAI Agents SDK for orchestration.

## Architecture

The backend consists of:
- FastAPI application serving the API endpoints
- OpenAI Agent for RAG orchestration
- Vector store integration with Qdrant Cloud
- Database integration with Neon Postgres
- Qwen embeddings for content vectorization

## Components

### Database Models
- `ChatSession`: Tracks user chat sessions
- `UserQuery`: Stores user questions
- `AgentResponse`: Stores AI-generated responses
- `RetrievedContent`: Links queries to relevant book content
- `BookContent`: Stores the book content with embeddings

### Services
- `vector_store.py`: Manages vector storage with Qdrant
- `embeddings.py`: Handles text embedding generation with Qwen models
- `document_ingestion.py`: Processes and indexes book content
- `agent.py`: Implements RAG agent with OpenAI Agents SDK
- `database.py`: Handles database connections and sessions

## Environment Variables

Create a `.env` file in the backend directory with the following variables:

```bash
# OpenRouter API Configuration
OPENROUTER_API_KEY=your_openrouter_api_key_here
OPENROUTER_MODEL_NAME=your_preferred_model_here  # e.g., "openai/gpt-4-turbo"

# Qdrant Vector Database Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=book_content

# Neon Postgres Database Configuration
DATABASE_URL=your_neon_postgres_connection_string_here

# Application Configuration
MAX_CONTEXT_TOKENS=2000
SESSION_RETENTION_DAYS=30
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=3600  # in seconds
```

## API Endpoints

- `POST /api/chat` - Process chat queries with RAG
- `POST /api/retrieve` - Perform content retrieval
- `GET /api/sessions/{session_id}` - Get session details
- `GET /health` - Health check

## Running the Application

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables in `.env`

3. Run the application:
```bash
python main.py
```

Or with uvicorn:
```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`.

## Endpoints

### Chat Endpoint
```bash
POST /api/chat
Content-Type: application/json

{
  "message": "Your question here",
  "session_id": "optional session ID",
  "mode": "full-book",  // or "selected-text"
  "selected_text": "optional selected text for selected-text mode"
}
```

### Content Retrieval
```bash
POST /api/retrieve
Content-Type: application/json

{
  "query": "Your search query",
  "mode": "full-book",  // or "selected-text"
  "selected_text": "optional selected text",
  "limit": 5
}
```