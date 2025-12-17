# Implementation Plan: Integrated RAG Chatbot SDK Alignment

## Technical Context

**System Overview**: An integrated RAG chatbot embedded in a Physical AI book website that uses OpenAI Agents SDK for backend orchestration and ChatKit SDK for frontend chat experience, with OpenRouter API as the LLM provider.

**Component Boundaries**:
- **Frontend**: Static site with ChatKit SDK integration
- **Backend**: FastAPI application with OpenAI Agents SDK
- **Vector Store**: Qdrant Cloud (free tier) for book content
- **Relational Storage**: Neon Serverless Postgres for metadata

**Technology Stack**:
- Backend: FastAPI
- Agent Orchestration: OpenAI Agents SDK
- Frontend Chat: OpenAI ChatKit SDK
- LLM Provider: OpenRouter API
- Embeddings: Qwen
- Vector DB: Qdrant Cloud
- Relational DB: Neon Serverless Postgres

**Architecture Sketch**:
```
[User Browser]
     ↓ (ChatKit SDK)
[Static Book Site]
     ↓ (API Call)
[FastAPI Backend] ←→ [Qdrant Cloud] (Vector search)
     ↓ (OpenRouter API)
[OpenRouter LLM]
     ↓ (DB Operations)
[Neon Postgres]
```

## Architecture Decisions

### Agent-based Orchestration vs Manual RAG Pipelines
- **Decision**: Use OpenAI Agents SDK for orchestration
- **Rationale**: Provides built-in tool calling, memory management, and conversation flow
- **Alternative**: Custom RAG pipeline with manual orchestration
- **Tradeoffs**: Less control vs faster development

### ChatKit SDK vs Custom Chat UI
- **Decision**: Use OpenAI ChatKit SDK for frontend
- **Rationale**: Provides pre-built components, message streaming, and session management
- **Alternative**: Custom React chat implementation
- **Tradeoffs**: Vendor lock-in vs faster development

### Server-side RAG vs Client-assisted Retrieval
- **Decision**: Server-side RAG with backend handling all retrieval
- **Rationale**: Better security, consistency, and control over context
- **Alternative**: Client-assisted retrieval with direct vector DB access
- **Tradeoffs**: Server load vs potential security concerns

### Embedding Model Choice (Qwen)
- **Decision**: Use Qwen embeddings
- **Rationale**: Cost-effective and compatible with OpenRouter
- **Alternative**: OpenAI embeddings
- **Tradeoffs**: Potential quality differences vs cost savings

### Vector DB Schema and Chunking Strategy
- **Decision**: Book content chunked by sections/chapters with metadata
- **Rationale**: Balances retrieval accuracy with context length
- **Alternative**: Sentence-level or paragraph-level chunks
- **Tradeoffs**: Context coherence vs precision

## Phase 0: Research & Resolution

### Research Tasks

1. **OpenAI Agents SDK Integration**
   - Validate compatibility with OpenRouter API
   - Research tool calling patterns for RAG
   - Document any limitations or workarounds

2. **ChatKit SDK Integration**
   - Research embedding in static sites
   - Validate customization options
   - Document limitations for book-specific UI

3. **Qwen Embeddings Performance**
   - Compare quality with OpenAI embeddings
   - Validate tokenization differences
   - Document any accuracy tradeoffs

4. **Qdrant Cloud Free Tier Limitations**
   - Document storage and query limits
   - Plan for scaling considerations
   - Validate required features are available

## Phase 1: Foundation

### Data Model

**Entities**:
- `ChatSession`: User chat session with history
- `UserQuery`: Individual user questions
- `RetrievedContent`: Book passages retrieved for context
- `AgentResponse`: LLM responses with citations
- `BookContent`: Vectorized book sections with metadata

**Relationships**:
- ChatSession contains multiple UserQuery-Response pairs
- UserQuery links to multiple RetrievedContent items
- BookContent has vector embeddings for similarity search

### API Contracts

**Backend Endpoints**:
- `POST /api/chat` - Create new chat session or continue existing
- `POST /api/retrieve` - Perform vector search on book content
- `POST /api/validate-context` - Ensure response grounding

**Frontend Integration**:
- ChatKit component embedded in book pages
- Mode selector (full-book vs selected-text)
- Context highlighting for retrieved content

## Phase 2: Integration

### Agent and Tool Orchestration

**Agent Configuration**:
- System prompt focused on Physical AI book content
- Tool for vector search in Qdrant
- Tool for context validation
- Memory management for conversation history

**Retrieval Tools**:
- Full-book search tool
- Selected-text search tool
- Context filtering based on mode

### Frontend ChatKit Integration

**Customization**:
- Book-specific styling
- Mode selection UI
- Content citation display
- Text selection integration

## Phase 3: Validation

### Quality Validation

**Context Isolation Correctness**:
- Verify selected-text mode only uses selected content
- Validate full-book mode properly retrieves relevant sections

**Retrieval Accuracy**:
- Measure precision of vector search
- Validate relevance of retrieved content

**Response Relevance**:
- Measure factual accuracy against book content
- Validate proper citation of sources

**Latency and Rate-limit Behavior**:
- Measure response times under load
- Validate rate limiting and caching strategies

## Security and Abuse Prevention

- Rate limiting per IP/session
- Query validation and sanitization
- Content filtering for inappropriate requests
- Session timeout and cleanup

## Observability and Logging

- Request/response logging for debugging
- Performance metrics collection
- Error tracking and alerting
- Usage analytics for product insights

## Testing Strategy

### Unit Tests
- Retrieval tools functionality
- Context filtering logic
- Agent response validation

### Integration Tests
- Agent → Tool → LLM flow
- End-to-end chat experience
- Database operations

### Frontend Tests
- ChatKit component behavior
- Mode switching functionality
- Text selection integration

### Specialized Tests
- Selected-text-only enforcement
- End-to-end RAG accuracy
- Cross-session data isolation