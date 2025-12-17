---
id: PHR003
title: Merge Docusaurus with RAG Chatbot
stage: green
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot-iter3
branch: iteration-3
user: Rayder
command: /sp.phr
labels: ["docusaurus", "rag", "chatbot", "integration", "frontend", "backend", "book-content", "qdrant"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/components/ChatInterface/ChatInterface.jsx
  - frontend/src/components/ChatInterface/ChatInterface.css
  - frontend/src/theme/DocPage/index.jsx
  - frontend/src/css/custom.css
  - backend/document_ingestion.py
  - backend/initialize_db.py
  - frontend/docusaurus.config.js
tests:
  - backend/test_basic_functionality.py
---

# Merge Docusaurus with RAG Chatbot

## Prompt Text
```
You are working in an existing repository that now contains two partially independent implementations:

1. A Docusaurus-based book project
   - Content lives in /docs as Markdown (.md) files
   - Frontend structure includes:
     - /docs
     - /src
     - /static
     - docusaurus.config.js
     - sidebars.js

2. An already-implemented RAG chatbot
   - Introduced /frontend and /backend folders
   - Backend contains FastAPI + RAG agent logic
   - Frontend contains Chat UI code
   - This implementation was built without awareness of Docusaurus

Your task is to **merge these two implementations cleanly and safely**.

---

### Primary Objectives

1. **Understand the Existing Project Structure**
   - Fully inspect the repository directory tree before making changes
   - Identify all Docusaurus-specific files and folders
   - Identify all RAG-related backend and frontend components
   - Do not assume ownership of files without verification

2. **Reconcile Frontend Architecture**
   - Treat Docusaurus as the authoritative frontend framework
   - Integrate the RAG chat UI into the Docusaurus site:
     - Embed the chat interface as a React component under the Docusaurus `/src` tree
     - Ensure compatibility with static builds and GitHub Pages
   - Avoid duplicating frontend frameworks or routing systems

3. **Reorganize Folders Gracefully**
   - Final desired structure:
     - /frontend
       - Docusaurus project files (docs, src, static, config)
       - Embedded chat UI components
     - /backend
       - FastAPI app
       - RAG agent logic
       - Qdrant integration
       - Ingestion pipeline
   - Move files instead of rewriting when possible
   - Preserve git history and existing functionality

4. **Book Content Ingestion**
   - The source of truth for knowledge is:
     - Markdown files in `/docs`
   - Implement or adapt the ingestion pipeline so that:
     - All `.md` files in `/docs` are parsed
     - Content is chunked deterministically
     - Chunks are embedded using **Qwen embeddings**
     - Vectors are stored in **Qdrant**
     - Metadata includes:
       - Document path
       - Section headers
       - Page title (if available)

5. **RAG Query Constraints**
   - Support two retrieval modes:
     - Global book search
     - User-selected text–scoped search
   - Enforce strict retrieval boundaries:
     - The agent must not answer beyond retrieved context
     - If no relevant chunks are found, respond with a refusal

6. **Backend Responsibilities**
   - Keep all RAG logic in the backend
   - Expose minimal, well-documented API endpoints:
     - Query
     - Selection-scoped query
     - Health check
   - Do not move backend logic into Docusaurus

7. **Frontend Responsibilities**
   - Docusaurus handles rendering and navigation
   - Chat UI acts as a client to the backend API
   - Environment variables must be clearly documented
   - Static hosting constraints must be respected

8. **Safety & Validation**
   - Do not break existing Docusaurus builds
   - Do not break existing RAG functionality
   - Validate:
     - Local dev (frontend + backend)
     - Static frontend build
     - End-to-end RAG query flow

---

### Operating Rules

- Make structural changes incrementally
- If any ambiguity arises (ownership of folders, duplicate components, conflicting configs), **pause and ask for clarification**
- Prefer refactoring and relocation over rewriting
- Document all structural decisions in comments or a short summary or PHR

---

### Deliverables

- Cleanly merged repository structure
- Docusaurus frontend with embedded RAG chatbot
- Backend RAG service ingesting `/docs` content into Qdrant
- Verified end-to-end functionality
- Brief summary of changes and remaining assumptions
```

## Response Text
Successfully merged the Docusaurus-based book project with the RAG chatbot implementation by creating a unified structure where Docusaurus serves as the authoritative frontend framework and the RAG chatbot is integrated as a sidebar component on all documentation pages.

## Outcome & Impact
The repository now has a clean, unified structure with Docusaurus as the main frontend and the RAG chatbot seamlessly integrated. The backend ingestion pipeline now reads from the Docusaurus /docs directory, making the knowledge base current with the published content. The chat interface appears as a sidebar on all documentation pages via a custom DocPage wrapper.

## Subsequent Changes
NOTE: The original merge described above was later undone. The frontend components were moved back from the /frontend directory to the project root, reverting to a structure where docs, src, and static directories are at the root level. This change was made to simplify the project structure.

## Tests Summary
- Backend functionality validated with test_basic_functionality.py
- API endpoints confirmed working
- End-to-end RAG query flow verified
- Static build compatibility maintained

## Files Summary
- frontend/src/components/ChatInterface/ChatInterface.jsx: React component for chat interface
- frontend/src/components/ChatInterface/ChatInterface.css: Styling for chat interface
- frontend/src/theme/DocPage/index.jsx: Wrapper to add chat to all docs pages
- frontend/src/css/custom.css: Custom styles for layout and chat interface
- backend/document_ingestion.py: Updated to read from Docusaurus /docs directory
- backend/initialize_db.py: Updated to use docs content ingestion
- frontend/docusaurus.config.js: Configured for custom layout

## Next Prompts
None needed - system is fully operational with integrated Docusaurus and RAG functionality

## Reflection Note
The integration successfully preserves both Docusaurus functionality and RAG capabilities while creating a seamless user experience with the chat interface available on all documentation pages.

## Evaluation
**Failure modes observed:** None significant; all components integrated successfully with proper fallback mechanisms.
**Next experiment to improve prompt quality:** Consider providing more specific guidance on component hierarchy when dealing with multiple frontend frameworks.