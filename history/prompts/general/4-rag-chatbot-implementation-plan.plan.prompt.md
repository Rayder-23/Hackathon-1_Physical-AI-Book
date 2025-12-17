---
id: 4
title: rag-chatbot-implementation-plan
stage: plan
date_iso: 2025-12-17
surface: agent
model: Claude Sonnet 4.5
feature: book-rag-chatbot
branch: 2-sdk-alignment
user: Ryyan
command: /sp.plan "Create: High-level system architecture for an Integrated RAG Chatbot embedded in a published book"
labels:
  - rag
  - chatbot
  - ai
  - architecture
  - planning
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/1-book-rag-chatbot/plan.md
  - specs/1-book-rag-chatbot/research.md
  - specs/1-book-rag-chatbot/data-model.md
  - specs/1-book-rag-chatbot/quickstart.md
  - specs/1-book-rag-chatbot/contracts/openapi.yaml
tests_yaml:
  - plan-validation
outcome: Comprehensive implementation plan created with architecture, data model, contracts, and research
evaluation: Plan addresses all requested components with clear phase structure and testing strategy
---

# PHR: rag-chatbot-implementation-plan

## User Input

```
Create:
- High-level system architecture for an Integrated RAG Chatbot embedded in a published book
- Component boundaries between frontend (static site), backend (FastAPI), vector store (Qdrant), and relational storage (Neon)
- RAG execution flow using OpenAI Agents SDK and retrieval tools
- Frontend chat embedding strategy using ChatKit SDK

Include:
1. Architecture Sketch
   - Static book frontend with embedded chatbot UI
   - FastAPI backend hosting Agent, tools, and retrieval pipeline
   - OpenRouter API as the LLM provider
   - Qwen embeddings for document and query embedding
   - Qdrant Cloud (free tier) for vector search
   - Neon Serverless Postgres for metadata, chat sessions, and audit logs

2. Section Structure
   - Data ingestion & indexing
   - Agent and tool orchestration
   - Retrieval modes:
     - Full-book context
     - User-selected text–only context
   - Frontend ChatKit integration
   - Security and abuse prevention
   - Observability and logging

3. Research Approach
   - Research concurrent with implementation
   - Validate SDK usage against official OpenAI and ChatKit documentation
   - Validate OpenRouter compatibility with OpenAI-compatible SDKs

4. Quality Validation
   - Context isolation correctness
   - Retrieval accuracy
   - Response relevance
   - Latency and rate-limit behavior

Decisions needing documentation:
- Agent-based orchestration vs manual RAG pipelines
- ChatKit SDK vs custom chat UI
- Server-side RAG vs client-assisted retrieval
- Embedding model choice (Qwen) tradeoffs
- Vector DB schema and chunking strategy

Testing strategy:
- Unit tests for retrieval tools
- Integration tests for Agent → Tool → LLM flow
- Frontend chat UI validation
- Selected-text-only enforcement tests
- End-to-end RAG accuracy checks

Technical details:
- Agent-driven RAG lifecycle
- OpenAI-compatible APIs via OpenRouter
- No backend-rendered UI
- Static-site-safe frontend integration
- Organize by phases:
  Research → Foundation → Integration → Validation
```

## Assistant Response

Created comprehensive implementation plan with all requested components:

- Implementation plan (plan.md) with architecture decisions and phase structure
- Research document (research.md) with findings on SDK compatibility and tradeoffs
- Data model (data-model.md) with entity definitions and relationships
- Quickstart guide (quickstart.md) with setup instructions
- API contracts (contracts/openapi.yaml) with OpenAPI specification

The plan addresses all requested components with a clear phase structure (Research → Foundation → Integration → Validation) and includes the specified testing strategy. All major architectural decisions are documented with alternatives and tradeoffs.

## Follow-up Actions

Ready for next phase: `/sp.adr SDK-Integration-Approach` to document key architectural decisions