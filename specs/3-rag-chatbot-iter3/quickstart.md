# Quickstart Guide: Integrated RAG Chatbot

## Prerequisites

- Python 3.9+ for backend services
- Node.js 18+ for frontend development
- OpenRouter API key with access to desired models
- Qdrant Cloud account and API key
- Neon Serverless Postgres account and connection string

## Environment Setup

### Backend (FastAPI)

1. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install dependencies:
```bash
pip install fastapi uvicorn openai python-dotenv qdrant-client sqlalchemy psycopg2-binary
```

3. Set up environment variables:
```bash
# .env file
OPENROUTER_API_KEY=your_openrouter_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_connection_string
```

### Frontend (Static Site)

1. Install ChatKit SDK:
```bash
npm install @openai/chatkit
```

2. Or include via CDN:
```html
<script src="https://cdn.openai.com/chatkit/client-js/v1/chatkit.min.js"></script>
```

## Backend Setup

### 1. Initialize Vector Store

```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Create collection for book content
client.recreate_collection(
    collection_name="book_content",
    vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),
)
```

### 2. Initialize Database Models

```python
from sqlalchemy import create_engine, Column, String, DateTime, Text, Float
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.dialects.postgresql import UUID
import uuid

Base = declarative_base()

class ChatSession(Base):
    __tablename__ = 'chat_sessions'
    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    # ... other fields as per data model
```

### 3. Create Agent with Tools

```python
from openai import OpenAI

client = OpenAI(base_url="https://openrouter.ai/api/v1", api_key=os.getenv("OPENROUTER_API_KEY"))

class RetrievalTool:
    @classmethod
    def get_definition(cls):
        return {
            "type": "function",
            "function": {
                "name": "retrieve_book_content",
                "description": "Retrieve relevant content from the Physical AI book",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {"type": "string", "description": "The search query"},
                        "mode": {"type": "string", "enum": ["full-book", "selected-text"]},
                        "selected_text": {"type": "string", "description": "Text selected by user (for selected-text mode)"}
                    },
                    "required": ["query", "mode"]
                }
            }
        }

    def __call__(self, query: str, mode: str, selected_text: str = None):
        # Implementation for retrieving content from Qdrant
        pass
```

## Frontend Integration

### 1. Embed ChatKit Component

```html
<div id="chat-container"></div>
<script>
  const chatkit = new ChatKit({
    // Configuration options
  });

  chatkit.mount({
    selector: '#chat-container',
    // Additional options
  });
</script>
```

### 2. Mode Selection UI

Create UI elements to switch between full-book and selected-text modes:

```html
<div class="chat-mode-selector">
  <button id="full-book-mode" class="active">Full Book Context</button>
  <button id="selected-text-mode">Selected Text Only</button>
</div>
```

## Running the Application

### Backend
```bash
uvicorn main:app --reload
```

### Frontend
For static site integration, include the ChatKit component in your existing HTML pages.

## API Endpoints

- `POST /api/chat`: Create or continue chat session
- `POST /api/retrieve`: Perform content retrieval
- `GET /api/sessions/{session_id}`: Get session details

## Testing

1. Index sample book content:
```bash
python scripts/index_book_content.py
```

2. Start the backend:
```bash
uvicorn main:app --reload
```

3. Visit your book page with the embedded chat component
4. Test both full-book and selected-text modes