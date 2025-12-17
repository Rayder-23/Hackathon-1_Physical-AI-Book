---
id: 2
title: integrated-rag-chatbot-sdk-alignment
stage: spec
date_iso: 2025-12-17
surface: agent
model: Claude Sonnet 4.5
feature: rag-chatbot-iter3
branch: iteration-3
user: Rayder
command: /sp.specify "Update spec.md: Integrated RAG Chatbot SDK Alignment"
labels:
  - rag
  - chatbot
  - ai
  - book-platform
  - sdk
  - openai
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - specs/1-book-rag-chatbot/spec.md
  - specs/1-book-rag-chatbot/checklists/requirements.md
tests_yaml:
  - spec-validation-checklist
outcome: Specification updated to incorporate OpenAI Agents SDK and ChatKit SDK while preserving existing architecture
evaluation: Specification meets quality standards with all checklist items validated
---

# PHR: integrated-rag-chatbot-sdk-alignment

## User Input

```
Update spec.md: Integrated RAG Chatbot SDK Alignment

Context:
A technical specification (`spec.md`) for an Integrated RAG Chatbot embedded in the Physical AI Book has already been created.

Goal of this update:
Revise and extend the existing specification to explicitly incorporate:
- OpenAI Agents SDK
- OpenAI ChatKit SDK
while preserving all previously approved architecture, constraints, and success criteria unless explicitly modified.

Scope of update:
1. SDK Integration Clarification
   - Define where and how OpenAI Agents SDK is used in the backend (FastAPI):
     - Agent orchestration
     - Tool calling for retrieval
     - Context enforcement (full-book vs selected-text)
   - Define where and how OpenAI ChatKit SDK is used in the frontend:
     - Chat UI state management
     - Message streaming
     - Session handling
     - Embedding into static site pages

2. API Provider Alignment
   - Clarify how OpenRouter API is used as the LLM provider beneath OpenAI-compatible SDKs
   - Document compatibility assumptions and limitations
   - Specify fallback or abstraction layers if SDK features are provider-dependent

3. RAG Flow Refinement
   - Update RAG lifecycle to reflect Agent-based orchestration:
     - User input → Agent → Retriever tool → Context filtering → Generation
   - Explicitly document "selected text only" enforcement at the agent/tool level

4. SDK-Specific Decisions
   - Justify choice of:
     - OpenAI Agents SDK vs custom orchestration
     - ChatKit vs custom React chat UI
   - Document tradeoffs (flexibility, vendor coupling, future extensibility)

5. Non-breaking Changes
   - Do not remove or invalidate:
     - Qdrant Cloud usage
     - Neon Serverless Postgres usage
     - Qwen embeddings
     - FastAPI backend
     - Static-site frontend constraints

Success criteria for the update:
- spec.md clearly explains the role of OpenAI Agents SDK and ChatKit SDK
- RAG orchestration is agent-driven and well-documented
- SDK usage is compatible with OpenRouter-based inference
- No ambiguity about frontend vs backend responsibilities
- Existing readers of the spec can understand what changed and why

Constraints:
- Update existing sections where possible instead of adding excessive new sections
- Use Markdown
- Maintain the original tone and structure of spec.md
- No implementation code beyond illustrative pseudocode
- No change to project timeline unless strictly required

Not building (explicitly reaffirmed: General-purpose chatbot unrelated to book content, Training or fine-tuning custom LLMs, Multi-modal RAG (images/audio/video), Paid enterprise infrastructure, Offline or desktop chatbot clients)
```

## Assistant Response

Updated the existing specification to incorporate OpenAI Agents SDK and ChatKit SDK while preserving all previously approved architecture. Key changes made:

- Updated the specification title and description to reflect the SDK alignment update
- Added new functional requirements (FR-011 to FR-018) covering SDK integration
- Added new key entities related to SDK components
- Added new success criteria (SC-009 to SC-013) for SDK-specific outcomes
- Added new "RAG Flow and Architecture" section explaining agent-driven orchestration
- Updated the checklist to reflect the changes made

The specification now clearly explains the role of both SDKs while maintaining compatibility with OpenRouter API and preserving all existing architectural constraints.

## Follow-up Actions

Ready for next phase: `/sp.clarify` or `/sp.plan`