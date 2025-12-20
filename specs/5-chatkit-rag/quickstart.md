# Quickstart: Chatkit-Based RAG Chatbot Integration

## Prerequisites

- Node.js 18+ and npm/yarn
- Python 3.11+
- Access to Qdrant Cloud instance
- Neon Postgres database
- OpenAI Chatkit account and credentials

## Setup Steps

### 1. Environment Configuration

Create `.env` file in the backend directory:

```env
CHATKIT_INSTANCE_LOCATOR=your_instance_locator
CHATKIT_KEY=your_secret_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_postgres_connection_string
```

### 2. Frontend Installation

```bash
# Install Chatkit React SDK
npm install @openai/chatkit-react

# Verify Docusaurus compatibility
npm install @docusaurus/core
```

### 3. Backend Installation

```bash
# Create Python virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python dependencies
pip install fastapi uvicorn openai-chatkit qdrant-client asyncpg python-dotenv
```

### 4. Database Setup

Run the database migrations using the schema defined in data-model.md:

```sql
-- Execute the SQL from data-model.md in your Neon Postgres database
```

### 5. Running the Application

#### Backend
```bash
cd backend
python main.py
```

#### Frontend
```bash
cd ..
npm run start
```

## Key Integration Points

### Frontend Component
```jsx
// src/components/Chatkit/ChatInterface.jsx
import { ChatkitProvider, MessageList, MessageInput } from '@openai/chatkit-react';

function ChatInterface() {
  return (
    <ChatkitProvider instanceLocator={instanceLocator} tokenProvider={tokenProvider} userId={userId}>
      <MessageList />
      <MessageInput />
    </ChatkitProvider>
  );
}
```

### Backend Service
```python
# backend/chatkit/adapter.py
from openai_chatkit import ServerInstance
from rag.query_processor import process_query

class ChatkitRAGAdapter:
    def __init__(self):
        self.chatkit_server = ServerInstance(
            instance_locator=os.environ['CHATKIT_INSTANCE_LOCATOR'],
            key=os.environ['CHATKIT_KEY']
        )

    async def handle_message(self, message):
        # Process message through RAG system
        response = await process_query(message.text)
        # Send response back to Chatkit room
        await self.chatkit_server.send_message(
            room_id=message.room_id,
            user_id='system_bot',
            text=response
        )
```

## Testing the Integration

1. Start both backend and frontend servers
2. Navigate to the Docusaurus site
3. Access the chat interface
4. Verify:
   - Chat messages appear in the interface
   - Queries are processed by the RAG system
   - Responses are returned in a timely manner
   - Session state is maintained appropriately
   - Rate limiting functions as expected

## Common Issues and Solutions

### Issue: Chatkit widget not loading
**Solution**: Verify instance locator and key are correct in environment variables

### Issue: RAG queries timing out
**Solution**: Check Qdrant connection and ensure documents are properly indexed

### Issue: Session not persisting
**Solution**: Verify Neon Postgres connection and session management logic

### Issue: Static site deployment problems
**Solution**: Ensure all Chatkit components work in static hosting environment