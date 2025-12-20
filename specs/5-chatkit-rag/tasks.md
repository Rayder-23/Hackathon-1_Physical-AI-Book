# Implementation Tasks: Chatkit-Based RAG Chatbot Integration

**Feature**: 5-chatkit-rag
**Date**: 2025-12-19
**Spec**: [spec.md](spec.md)
**Plan**: [plan.md](plan.md)
**Input**: User stories from spec.md with priorities (P1, P2, P3, etc.)

## Dependencies & Parallel Execution

**User Story Dependency Graph**:
- US1 (P1) and US2 (P1) can be implemented in parallel
- US3 (P2) depends on US1 and US2 completion

**Parallel Execution Opportunities**:
- T001-T004: Setup and research tasks can run in parallel
- T010-T015: Frontend component creation can run in parallel
- T020-T025: Backend component creation can run in parallel

## Implementation Strategy

**MVP Scope**: User Story 1 only (core chat functionality with Physical AI Book content)
**Delivery Approach**: Incremental - implement and test each user story independently before moving to the next
**Testing**: Each user story should be independently testable per its acceptance criteria

---

## Phase 1: Setup & Research Tasks

### Story Goal
Prepare development environment and understand requirements before implementation.

### Independent Test Criteria
- All required documentation has been reviewed
- Existing chatbot code has been audited and cataloged
- Development environment is ready for implementation

### Tasks

- [x] T001 Research Chatkit JS SDK documentation and understand React widget patterns
- [x] T002 Research Chatkit Python SDK documentation and understand backend integration
- [x] T003 Audit existing chatbot frontend components and identify files to be replaced
- [x] T004 Audit existing chatbot backend code and identify files to be updated
- [x] T005 [P] Install frontend dependency: npm install @openai/chatkit-react
- [x] T006 [P] Install backend dependency: pip install openai-chatkit
- [x] T007 [P] Verify compatibility of Chatkit dependencies with existing project structure

---

## Phase 2: Foundational Tasks

### Story Goal
Set up foundational infrastructure needed for all user stories.

### Independent Test Criteria
- Database schemas for sessions, conversations, and RAG queries are created
- Basic backend API structure is in place
- Authentication integration points are established

### Tasks

- [x] T008 Create database migration files for sessions, conversations, and rag_queries tables in Neon Postgres
- [x] T009 [P] Create User Session model in backend/models/models.py based on data-model.md
- [x] T010 [P] Create Guest Session model in backend/models/models.py based on data-model.md
- [x] T011 [P] Create Chat Message model in backend/models/models.py based on data-model.md
- [x] T012 [P] Create Conversation Thread model in backend/models/models.py based on data-model.md
- [x] T013 [P] Create RAG Query model in backend/models/models.py based on data-model.md
- [x] T014 [P] Set up database connection and session management in backend/database/
- [x] T015 Create session management service in backend/services/session_service.py
- [x] T016 [P] Create Chatkit adapter interface in backend/chatkit/adapter.py
- [x] T017 [P] Create Qdrant client integration in backend/data/vector_store.py
- [x] T018 [P] Create query processor in backend/agents/agent.py
- [x] T019 [P] Create document loader for RAG in backend/data/document_ingestion.py

---

## Phase 3: [US1] Chat with Physical AI Book Content

### Story Goal
As a reader of the Physical AI book, I want to interact with an intelligent chatbot that can answer questions about the book content, so I can get immediate clarifications and deeper understanding of complex topics.

### Independent Test Criteria
A user can successfully ask a question about the book content and receive a relevant, accurate response from the chatbot within 5-10 seconds.

### Acceptance Scenarios
1. Given a user is viewing the Physical AI book website, When they interact with the Chatkit-powered chatbot interface and ask a question about the book content, Then they receive a relevant response based on the book's content and context.
2. Given a user has been chatting with the chatbot, When they ask a follow-up question that references previous conversation, Then the chatbot maintains context and provides coherent responses.

### Tasks

- [x] T020 [P] [US1] Create ChatkitProvider component in src/components/Chatkit/ChatkitProvider.jsx
- [x] T021 [P] [US1] Create ChatkitWidget component in src/components/Chatkit/ChatkitWidget.jsx
- [x] T022 [P] [US1] Create ChatInterface component in src/components/Chatkit/ChatInterface.jsx
- [x] T023 [P] [US1] Create chat styling in src/css/chatkit-styles.css
- [x] T024 [P] [US1] Integrate Chatkit components into Docusaurus layout as persistent sidebar
- [x] T025 [US1] Create backend route for Chatkit token generation in backend/chatkit/routes.py
- [x] T026 [US1] Implement Chatkit message handler in backend/chatkit/adapter.py
- [x] T027 [US1] Connect RAG backend to Chatkit adapter for query processing
- [x] T028 [US1] Implement context maintenance for follow-up questions using conversation history in backend/agents/agent.py
- [x] T029 [US1] Add loading indicators during query processing per FR-010
- [x] T030 [US1] Test end-to-end chat functionality with Physical AI Book content

---

## Phase 4: [US2] Access Chatbot from Any Page

### Story Goal
As a reader browsing the Physical AI book website, I want to access the chatbot functionality from any page, so I can get help without navigating to a specific location.

### Independent Test Criteria
The chatbot interface is available on all pages of the Docusaurus site and functions consistently across different page types.

### Acceptance Scenarios
1. Given a user is on any page of the Physical AI book website, When they click on the chatbot interface, Then the Chatkit widget opens and is ready for interaction.
2. Given a user has an active chat session, When they navigate to a different page, Then their conversation context is preserved.

### Tasks

- [x] T031 [P] [US2] Modify Docusaurus theme to include Chatkit widget on all pages
- [x] T032 [US2] Implement session persistence across page navigation in frontend
- [x] T033 [US2] Create Chatkit context provider to maintain state across pages
- [x] T034 [US2] Test chatbot availability on various Docusaurus page types
- [x] T035 [US2] Verify conversation context preservation during page navigation

---

## Phase 5: [US3] Persistent Chat Sessions

### Story Goal
As a reader who uses the chatbot across multiple visits, I want my chat history to be preserved, so I can continue conversations where I left off.

### Independent Test Criteria
A user can close the browser, return later, and continue their previous conversation with access to chat history.

### Acceptance Scenarios
1. Given a user has an active chat session, When they close the browser and return later, Then they can resume their conversation or view their chat history.
2. Given a user is logged in to their account, When they access the chatbot from different devices, Then their conversation history is synchronized.

### Tasks

- [x] T036 [US3] Implement authenticated user session handling with Neon Postgres
- [x] T037 [US3] Implement guest session handling with time-based expiration (24 hours)
- [x] T038 [US3] Create conversation history API endpoints in backend/chatkit/routes.py
- [x] T039 [US3] Implement session synchronization between devices for authenticated users
- [x] T040 [US3] Test session persistence across browser restarts
- [x] T041 [US3] Test conversation history access for returning users

---

## Phase 6: Error Handling & Validation

### Story Goal
Implement error handling, rate limiting, and validation to ensure system reliability.

### Independent Test Criteria
System handles errors gracefully, implements rate limiting, and maintains data integrity.

### Tasks

- [x] T042 Implement circuit breaker pattern between Chatkit and RAG services per research.md
- [x] T043 Add rate limiting for chat requests per FR-012 in backend/utils/rate_limiting.py
- [x] T044 Implement error handling and user-friendly messages per FR-011
- [x] T045 Add logging of technical metadata (not conversation content) per FR-013
- [x] T046 Validate response formatting for Chatkit compatibility
- [x] T047 Test error handling when Qdrant vector store is unavailable
- [x] T048 Test error handling when Neon Postgres database has connectivity issues

---

## Phase 7: Polish & Cross-Cutting Concerns

### Story Goal
Complete implementation by removing old code, validating static site compatibility, and documenting changes.

### Independent Test Criteria
No custom chatbot UI remains, static site builds successfully, and all requirements are met.

### Tasks

- [x] T049 Remove old custom chatbot components identified in audit (src/components/ChatInterface/)
- [x] T050 Remove old custom chatbot pages identified in audit (src/pages/chat/)
- [x] T051 Verify static site build and deployment compatibility with GitHub Pages
- [x] T052 Test end-to-end chat flow including all user stories
- [x] T053 Document all replaced or updated files in documentation
- [x] T054 Validate that Chatkit UI is fully implemented using @openai/chatkit-react exclusively (no custom UI code)
- [x] T055 Confirm backend uses openai-chatkit Python SDK to connect Chatkit to the RAG system
- [x] T056 Verify chatbot correctly answers questions using Qdrant vector store
- [x] T057 Validate user sessions are managed through Neon Postgres
- [x] T058 Run full test suite to confirm all success criteria are met
- [x] T059 Update documentation to reflect new Chatkit-based implementation