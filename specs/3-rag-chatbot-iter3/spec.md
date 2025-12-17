# Feature Specification: Integrated RAG Chatbot for Physical AI Book Platform (SDK Alignment Update)

**Feature Branch**: `rag-chatbot-iter3`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Integrated RAG Chatbot for Physical AI Book Platform (Iteration 3.0)

Target audience:
Readers of the published Physical AI book (students, developers, researchers) who want interactive, contextual explanations of the book content.

Focus:
Design and implementation of a Retrieval-Augmented Generation (RAG) chatbot embedded into the book website that can:
- Answer questions about the full book content
- Answer questions based strictly on user-selected text
- Operate within a static-site frontend with a hosted backend

Updated Focus (SDK Alignment):
- Incorporate OpenAI Agents SDK for backend orchestration
- Incorporate OpenAI ChatKit SDK for frontend chat interface
- Maintain compatibility with OpenRouter API as LLM provider

Success criteria:
- Chatbot answers questions grounded strictly in retrieved book content
- Supports two query modes:
  - Full-book context
  - User-selected text only
- Uses OpenRouter API for LLM inference through OpenAI-compatible SDKs
- Uses Qwen embeddings for vectorization
- Uses Qdrant Cloud (Free Tier) for vector storage
- Uses Neon Serverless Postgres for metadata, chat sessions, and auth linkage
- Embedded seamlessly into the published book UI
- Clear separation between frontend (static) and backend (API)
- Leverages OpenAI Agents SDK for orchestration and tool calling
- Leverages OpenAI ChatKit SDK for frontend chat experience

Constraints:
- Frontend: Static site (no SSR dependency at runtime)
- Backend: FastAPI only
- Vector DB: Qdrant Cloud Free Tier
- Database: Neon Serverless Postgres
- LLM access: OpenRouter API through OpenAI-compatible SDKs
- Embeddings: Qwen
- SDKs: OpenAI Agents SDK (backend), OpenAI ChatKit SDK (frontend)
- Output format: Technical specification in Markdown
- Timeline: 2–3 weeks

Not building:
- General-purpose chatbot unrelated to book content
- Training or fine-tuning custom LLMs
- Multi-modal RAG (images/audio/video)
- Paid enterprise infrastructure
- Offline or desktop chatbot clients"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Full-book Context Query (Priority: P1)

As a reader of the Physical AI book, I want to ask questions about the book content so that I can get contextual explanations and clarifications on complex topics.

**Why this priority**: This is the core functionality that provides immediate value to users by enabling them to interact with the entire book content for comprehensive answers.

**Independent Test**: Can be fully tested by submitting a question to the chatbot and receiving an answer grounded in the book content, delivering immediate value for understanding complex topics.

**Acceptance Scenarios**:

1. **Given** a user has access to the book content, **When** they submit a question about the book, **Then** the chatbot responds with an answer based on relevant passages from the full book content
2. **Given** a user submits a question, **When** the system retrieves relevant book content, **Then** the response cites the relevant sections or chapters from the book

---

### User Story 2 - User-Selected Text Query (Priority: P2)

As a student studying a specific section of the Physical AI book, I want to ask questions about only the text I have selected so that I can get focused explanations on that particular content.

**Why this priority**: This provides a more focused interaction mode that allows users to get answers based on specific sections they are currently reading.

**Independent Test**: Can be tested by selecting text in the book interface and asking a question, delivering value by providing answers strictly from the selected text.

**Acceptance Scenarios**:

1. **Given** a user has selected specific text in the book, **When** they submit a question about that text, **Then** the chatbot responds with answers grounded only in the selected text
2. **Given** a user has selected text, **When** they activate the text-only query mode, **Then** the chatbot ignores the broader book context and responds only based on the selected text

---

### User Story 3 - Chat Session Management (Priority: P3)

As a returning user, I want to maintain my chat history across sessions so that I can continue conversations about the book content.

**Why this priority**: This enhances the user experience by allowing continuity in learning and exploration of the book content.

**Independent Test**: Can be tested by starting a conversation, leaving the site, and returning to see the chat history preserved, delivering value by maintaining learning context.

**Acceptance Scenarios**:

1. **Given** a user has an active chat session, **When** they close the browser and return later, **Then** they can access their previous conversation history
2. **Given** a user is logged in, **When** they access the chatbot from different devices, **Then** they can view their chat history

---

### Edge Cases

- What happens when the selected text is too short or contains no meaningful content?
- How does the system handle queries when the book content is temporarily unavailable?
- What occurs when a user submits a query that has no relevant matches in the book content? (Answer: System returns "No relevant content found")
- How does the system handle extremely long user selections or very complex queries?
- What happens when the vector database is temporarily unreachable?
- How are anonymous user sessions handled differently from authenticated sessions?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to submit natural language questions about the Physical AI book content
- **FR-002**: System MUST provide answers grounded in the retrieved book content with appropriate citations
- **FR-003**: System MUST support two distinct query modes: full-book context and user-selected text only
- **FR-004**: System MUST retrieve relevant book content using vector similarity search
- **FR-004.1**: System MUST limit full-book context retrieval to maximum 2000 tokens
- **FR-005**: System MUST generate human-readable responses based on the retrieved content
- **FR-006**: System MUST maintain chat session history for authenticated users and temporary sessions for anonymous users
- **FR-006.1**: System MUST retain chat history for 30 days before automatic cleanup
- **FR-007**: System MUST provide a seamless integration with the existing book website UI
- **FR-008**: System MUST validate that responses are factually consistent with the source material
- **FR-009**: System MUST handle concurrent users without performance degradation
- **FR-010**: System MUST log user interactions for analytics and improvement purposes
- **FR-011**: System MUST utilize OpenAI Agents SDK for backend orchestration and tool calling
- **FR-012**: System MUST enforce context filtering (full-book vs selected-text) through agent tools
- **FR-012.1**: System MUST restrict retriever to user-selected text only in selected-text mode
- **FR-012.2**: System MUST return "No relevant content found" when no matches exist in the book content
- **FR-013**: System MUST integrate OpenAI ChatKit SDK for frontend chat UI and state management
- **FR-014**: System MUST maintain compatibility with OpenRouter API through OpenAI-compatible interfaces
- **FR-015**: System MUST implement proper abstraction layers to handle potential SDK/provider incompatibilities
- **FR-016**: Backend MUST orchestrate RAG flow as: User input → Agent → Retriever tool → Context filtering → Generation
- **FR-017**: Frontend MUST manage chat state, message streaming, and session handling via ChatKit SDK
- **FR-018**: System MUST ensure "selected text only" mode is enforced at the agent/tool level

### Key Entities *(include if feature involves data)*

- **Chat Session**: Represents a conversation between a user and the chatbot, containing multiple exchanges
- **Query**: A user's question submitted to the system, including metadata about the query mode used
- **Retrieved Content**: Book passages retrieved by the system to ground the response, with relevance scores
- **Response**: The chatbot's answer generated based on retrieved content, including citations
- **User Profile**: Information linking chat sessions to authenticated users for history persistence
- **Book Content**: The source material from the Physical AI book, segmented and stored in vector format
- **Agent Orchestration**: Backend component using OpenAI Agents SDK to manage conversation flow and tool calling
- **Retriever Tool**: Specialized tool within the agent that performs vector similarity searches against book content
- **Chat Interface**: Frontend component using OpenAI ChatKit SDK for message display, input handling, and streaming
- **API Abstraction Layer**: Middleware component ensuring compatibility between OpenAI SDKs and OpenRouter API

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can receive relevant answers to their questions within 5 seconds of submission
- **SC-002**: 90% of user queries result in responses that are factually consistent with the book content
- **SC-003**: Users engage in conversations averaging 3+ exchanges per session, demonstrating sustained interest
- **SC-004**: The system successfully handles 100+ concurrent users without performance degradation
- **SC-005**: 85% of users report improved understanding of book content after using the chatbot
- **SC-006**: The chatbot achieves a factual accuracy rate of 95% when citing book content
- **SC-007**: Response relevance is rated as "helpful" or "very helpful" by 80% of users
- **SC-008**: The system maintains 99% uptime during peak usage hours
- **SC-009**: The OpenAI Agents SDK successfully orchestrates the RAG workflow with 98% reliability
- **SC-010**: The OpenAI ChatKit SDK provides a seamless chat experience with sub-200ms message rendering
- **SC-011**: The system maintains compatibility with OpenRouter API while using OpenAI SDK interfaces
- **SC-012**: Context filtering (full-book vs selected-text) is correctly enforced in 99% of queries
- **SC-013**: The API abstraction layer handles SDK/provider incompatibilities without service disruption

## RAG Flow and Architecture

### Agent-Driven RAG Orchestration

The RAG lifecycle now follows an agent-driven approach:

1. **User Input**: User submits a question via the frontend ChatKit interface
2. **Agent Orchestration**: OpenAI Agents SDK processes the input and determines the appropriate tools to call
3. **Retriever Tool**: Specialized tool performs vector similarity search against book content in Qdrant Cloud
4. **Context Filtering**: Agent enforces the selected query mode (full-book vs selected-text only) through tool parameters
5. **Generation**: Agent generates response using OpenRouter API via the OpenAI-compatible interface
6. **Response Delivery**: Response is streamed through ChatKit SDK to the frontend

### SDK-Specific Implementation

**OpenAI Agents SDK (Backend)**:
- Provides orchestration capabilities for the RAG workflow
- Handles tool calling for content retrieval from Qdrant Cloud
- Enforces context filtering between full-book and selected-text modes
- Manages conversation state and memory within the agent

**OpenAI ChatKit SDK (Frontend)**:
- Provides pre-built chat interface components for seamless user experience
- Handles message streaming and real-time updates
- Manages chat session state and history in the browser
- Integrates cleanly with static site architecture

### API Provider Alignment

The system maintains compatibility with OpenRouter API through OpenAI-compatible interfaces:
- OpenAI Agents SDK calls are translated to OpenRouter-compatible API requests
- Proper abstraction layers handle any feature gaps between OpenAI and OpenRouter
- Fallback mechanisms ensure service continuity if specific features are not supported

## Clarifications

### Session 2025-12-17

- Q: Should users be authenticated to access persistent chat history? → A: Anonymous users get temporary chat sessions, auth required for permanent history
- Q: What should be the maximum context retrieval for full-book queries? → A: Full-book queries should retrieve maximum 2000 tokens of context
- Q: How should selected text only mode be enforced? → A: Selected text mode enforced by restricting retriever to user-selected text only
- Q: How long should chat history be retained? → A: Chat history should be retained for 30 days before cleanup
- Q: What should happen when no relevant content is found? → A: Return "No relevant content found" when no matches exist