---
id: 4
title: Physical AI Book Iteration 2 Planning
stage: plan
date: 2025-12-08
surface: agent
model: Claude Sonnet 4.5
feature: physical-ai-book-iteration2
branch: 2-physical-ai-book-iteration2
user: Claude Code User
command: /sp.plan
labels: ["physical-ai", "docusaurus", "planning", "architecture", "authentication"]
links:
  spec: specs/2-physical-ai-book-iteration2/spec.md
  plan: specs/2-physical-ai-book-iteration2/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/2-physical-ai-book-iteration2/plan.md
 - specs/2-physical-ai-book-iteration2/research.md
 - specs/2-physical-ai-book-iteration2/data-model.md
 - specs/2-physical-ai-book-iteration2/quickstart.md
 - specs/2-physical-ai-book-iteration2/contracts/auth-api.yaml
tests:
 - plan completeness
 - research validation
 - data model accuracy
---

## Prompt

Physical AI Book Technical Implementation Plan (Iteration 2)

Create:
- Architecture sketch for the full Docusaurus system (site structure, routing, auth flows, MCP automation)
- Section structure for docs, author area, personalization controls, and translation layers
- Research approach for technical decisions (Docusaurus official docs, better-auth docs, GH Pages deployment docs)
- Quality validation plan covering build integrity, deployment reliability, auth correctness, and MCP tooling stability

Decisions needing documentation:
- Docusaurus file restructuring: keep or remove blog, layout of modules, MDX conventions
- Authentication model: static-only better-auth vs alternative patterns; token storage strategy; client-side guards
- Data handling for personalization: where to store user background info; how to adapt MDX output
- Translation design: pre-generated Urdu MDX vs on-demand MCP generation
- GitHub Pages deployment path: root vs project subdirectory; branch strategy
- MCP usage strategy: auto-scaffold vs manual; directory extraction vs explicit config files

Testing strategy:
Validation checks must align with acceptance criteria defined in the specification:
- Docusaurus build: must pass without warnings; links and sidebar must resolve correctly
- GitHub Pages deployment: site must load at correct URL with baseUrl functioning
- Authentication:
  - author-only pages require login
  - signup flow stores user background locally (better-sqlite3 in author mode)
  - tokens persist only client-side for logged users
- Personalization:
  - personalized mode modifies chapter content when toggle is activated
  - fallback content loads correctly when disabled
- Urdu translation:
  - translation toggle loads Urdu MDX without breaking navigation
  - all Urdu files validated through MCP for MDX syntax and frontmatter correctness
- MCP automation:
  - scaffolding produces correct directory structure
  - sidebar auto-generation produces valid sidebars.js
  - translation and personalization content generation yields compilable MDX

Technical details:
- Use a research-concurrent workflow: consult Docusaurus docs and better-auth docs during implementation
- Automatically integrate MCP servers for file generation, structure extraction, and MDX validation
- All patterns must follow technical constraints of static hosting + GitHub Pages
- Organize by implementation phases:
  1. Research → gather Docusaurus, better-auth, and GH Pages specifics
  2. Foundation → restructure template, clean src/static, establish docs/ architecture
  3. Analysis → design auth flow, personalization logic, translation toggle, MCP automation scripts
  4. Synthesis

## Response snapshot

Successfully completed the planning phase for Physical AI Book (Iteration 2) with the following deliverables:

1. **Implementation Plan** (plan.md): Comprehensive architecture plan outlining the Docusaurus system structure, technical context, and project organization with proper compliance to static hosting constraints.

2. **Research Document** (research.md): Detailed research covering all key technical decisions including:
   - Docusaurus file restructuring and MDX conventions
   - Static-only better-auth implementation with client-side guards
   - User background storage in browser localStorage
   - Pre-generated Urdu MDX translation approach
   - GitHub Pages deployment strategy
   - MCP server integration for automation

3. **Data Model** (data-model.md): Complete entity definitions for User Profile, Book Module, Chapter Content, Translation Pair, and Access Level with relationships and validation rules.

4. **Quickstart Guide** (quickstart.md): Step-by-step setup instructions covering installation, development, and deployment workflows.

5. **API Contracts** (contracts/auth-api.yaml): OpenAPI specification for authentication and personalization endpoints (for client-side API interactions).

All deliverables align with the specification requirements and technical constraints, particularly the static hosting requirement, better-auth in static mode, and pre-generated Urdu translation approach.

## Outcome

- ✅ Impact: Complete technical implementation plan created with research, data model, quickstart guide, and API contracts
- 🧪 Tests: All planning artifacts validated for completeness and alignment with requirements
- 📁 Files: 5 planning artifacts created in specs/2-physical-ai-book-iteration2/
- 🔁 Next prompts: Ready for task generation and implementation phases
- 🧠 Reflection: Successfully planned the Physical AI Book iteration 2 with all required technical considerations

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All planning artifacts successfully created
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to task generation phase