# Implementation Plan: [FEATURE]

**Branch**: `5-chatkit-rag` | **Date**: 19-12-2025 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/5-chatkit-rag/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Chatkit-based RAG chatbot for the Physical AI Book website. The solution integrates @openai/chatkit-react widgets into the Docusaurus frontend while connecting to an existing RAG backend through the openai-chatkit Python SDK. The system will support both authenticated users (with persistent sessions in Neon Postgres) and guest users (with temporary sessions), providing the same quality of responses to all users. The implementation follows a three-tier architecture with clear separation between frontend, backend, and data layers, ensuring static site compatibility for GitHub Pages deployment.

## Technical Context

**Language/Version**: JavaScript/TypeScript (frontend), Python 3.11+ (backend)
**Primary Dependencies**: @openai/chatkit-react (frontend), openai-chatkit Python SDK (backend), Docusaurus, React, Qdrant client, Neon Postgres driver
**Storage**: Neon Postgres (user sessions), Qdrant Cloud (vector store for RAG)
**Testing**: Jest (frontend), pytest (backend)
**Target Platform**: Web browser (Docusaurus site), compatible with static hosting (GitHub Pages)
**Project Type**: Web application (frontend Docusaurus React components + backend Python RAG system)
**Performance Goals**: <5s response time for queries, 99% uptime for chat functionality
**Constraints**: Static site compatibility (no server-side rendering dependencies), guest session time limits (24 hours), rate limiting (requests per time period)
**Scale/Scope**: Support for multiple concurrent users accessing the chatbot simultaneously

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Technical Accuracy**: All dependencies and technologies mentioned (Chatkit, Docusaurus, Qdrant, Neon Postgres) are verified real technologies with official documentation. API contracts follow OpenAPI 3.0 specification standards.

**Clarity**: The plan uses clear, accessible language appropriate for the development team and maintains consistency with existing project terminology. All technical concepts are explained with appropriate context.

**Consistency**: The implementation follows existing project structure and coding patterns established in the Docusaurus site. The three-tier architecture aligns with standard web application patterns.

**Reliability**: All code examples and API contracts are designed to be tested and validated to ensure they function correctly within the Docusaurus environment. Error handling and fallback mechanisms are specified.

**Documentation Style**: The implementation follows high-quality technical documentation standards with proper structure and formatting. All documentation adheres to the project's established templates and patterns.

**Content Authenticity**: All implementation details are based on official Chatkit documentation and verified technical capabilities, with no hallucinated features or APIs. All external dependencies are real and properly referenced.

## Project Structure

### Documentation (this feature)

```text
specs/5-chatkit-rag/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application (frontend Docusaurus React components + backend Python RAG system)

# Frontend (Docusaurus/React components)
src/
├── components/
│   └── Chatkit/
│       ├── ChatkitWidget.jsx      # Main Chatkit widget integration
│       ├── ChatkitProvider.jsx    # Chatkit context provider
│       └── ChatInterface.jsx      # Chat interface wrapper
├── pages/
│   └── chat/
│       └── ChatPage.jsx           # Dedicated chat page (if needed)
└── css/
    └── chatkit-styles.css         # Chatkit widget styling

# Backend (Python RAG system)
backend/
├── main.py                        # FastAPI application entry point
├── __init__.py                    # Package initialization
├── agents/
│   ├── __init__.py
│   └── agent.py                   # RAG agent implementation and query processing
├── auth/
│   ├── __init__.py
│   ├── models.py                  # Authentication-related models
│   ├── routes.py                  # Authentication API routes
│   └── services.py                # Authentication services
├── chatkit/
│   ├── __init__.py
│   ├── adapter.py                 # Chatkit Python SDK adapter
│   └── routes.py                  # Chatkit API routes
├── data/
│   ├── __init__.py
│   ├── vector_store.py            # Qdrant vector store integration
│   ├── embeddings.py              # Embedding generation services
│   ├── document_ingestion.py      # Document loading for RAG
│   └── validation.py              # Data validation utilities
├── database/
│   ├── __init__.py
│   ├── database.py                # Database connection and session management
│   └── migrations/                # Alembic migration files
├── models/
│   ├── __init__.py
│   └── models.py                  # Database models and schema definitions
├── services/
│   ├── __init__.py
│   └── session_service.py         # Neon Postgres session management
└── utils/
    ├── __init__.py
    └── rate_limiting.py           # Rate limiting utilities

# Existing files to be replaced/updated
src/
├── components/
│   └── Chat/                      # Any existing custom chat components to be removed
└── pages/
    └── chat/                      # Any existing custom chat pages to be removed

# Backend files to be updated for new structure
backend/
├── agents/agent.py                # Updated RAG agent with new import paths
├── data/vector_store.py           # Updated vector store with new import paths
├── data/embeddings.py             # Updated embeddings with new import paths
├── data/document_ingestion.py     # Updated document ingestion with new import paths
├── database/database.py            # Updated database connection with new import paths
├── models/models.py                # Updated models with new import paths
├── services/session_service.py    # Updated session service with new import paths
├── chatkit/adapter.py             # Updated adapter with new import paths
├── chatkit/routes.py               # Updated routes with new import paths
└── main.py                        # Updated main app with new import paths
```

**Structure Decision**: The implementation follows a web application structure with separate frontend components for Chatkit integration in the Docusaurus site and backend Python services for RAG functionality. The existing custom chat components will be replaced with official Chatkit widgets as specified in the requirements.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
