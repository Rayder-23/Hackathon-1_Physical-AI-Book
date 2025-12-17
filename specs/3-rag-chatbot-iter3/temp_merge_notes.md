# Repository Structure Before Merge

## Current Structure:
- Root directory contains Docusaurus project files:
  - /docs (book content)
  - /src (Docusaurus source)
  - /static (Docusaurus static assets)
  - docusaurus.config.js, sidebars.js
  - package.json, etc.
- /backend (RAG chatbot backend)
- /frontend (standalone HTML chat interface - CONFLICT!)

## Desired Structure:
- /docs
- /src
- /static
- docusaurus.config.js
- sidebars.js
- etc.
- /backend (RAG backend)
  - Current backend content

## Plan:
1. Move Docusaurus files to new /frontend directory
2. Integrate RAG chat UI as React component in Docusaurus /src
3. Update ingestion to read from /frontend/docs
4. Keep backend separate