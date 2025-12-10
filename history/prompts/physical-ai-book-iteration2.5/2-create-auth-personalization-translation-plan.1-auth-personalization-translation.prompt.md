---
id: 2
title: create-auth-personalization-translation-plan
stage: plan
date: 2025-12-10
surface: agent
model: Claude Sonnet 4.5
feature: physical-ai-book-iteration2.5
branch: "iteration-2.5"
user: Claude
command: /sp.plan
labels:
  - authentication
  - personalization
  - translation
  - docusaurus
  - static-hosting
  - better-auth
links:
  spec: ../physical-ai-book-iteration2.5/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/physical-ai-book-iteration2.5/plan.md
  - specs/physical-ai-book-iteration2.5/research.md
  - specs/physical-ai-book-iteration2.5/data-model.md
  - specs/physical-ai-book-iteration2.5/quickstart.md
  - specs/physical-ai-book-iteration2.5/contracts/api-contracts.md
tests: []
---

# Prompt History Record

## PROMPT_TEXT

```
Create:
- Architecture sketch for authentication, personalization state, and translation switching in a static Docusaurus site.
- Section structure: Auth integration updates, header modifications, translation asset pipeline, global state logic.
- Research approach: Review Better-Auth documentation, Docusaurus client-only patterns, GitHub Pages deployment constraints.
- Quality validation: Ensure SSR safety, verify no broken builds, confirm translation toggle works globally.

Decisions needing documentation:
1. Auth strategy options:
   - A: Better-Auth client-only (chosen) — static, SSR-safe, compatible with GitHub Pages.
   - B: Remote auth API (rejected) — violates static site deployment constraints.
   Tradeoff: Client-only auth is simpler but limited (no server persistence).

2. Translation storage format:
   - A: Static JSON under /static/i18n (chosen) — fast load, easy toggle.
   - B: MDX duplication for Urdu (rejected) — maintenance overhead.
   Tradeoff: JSON requires mapping text segments, MDX duplication is heavier.

3. State management:
   - A: React Context via Docusaurus Root.js wrapper (chosen) — global availability.
   - B: Per-page local state (rejected) — inconsistent UX.
   Tradeoff: Context adds slight complexity but ensures stability.

4. Header UI:
   - A: Conditional rendering based on auth state (chosen).
   - B: Separate pages for toggle actions (rejected).
   Tradeoff: Inline UI is more intuitive and requires less navigation.

Testing strategy:
- Validate Better-Auth integration with example workflows (register → login → logout).
- Confirm SSR safety: "npm run build" must produce zero errors and zero hydration mismatches.
- Verify translation JSON loads correctly and toggles on any page globally.
- Test header rendering in both states (logged-out and logged-in).
- Check that GitHub Pages deploy works with no broken paths.
- Ensure personalized UI appearance is consistent across page reloads.

Technical details:
- Use research-concurrent approach (refer to Better-Auth + Docusaurus docs while implementing).
- Maintain specs defined in the Constitution (accuracy, clarity, reproducibility).
- Organize work in phases:
  Phase 1: Research → Better-Auth client-only patterns + Docusaurus SSR constraints
  Phase 2: Foundation → Build global context + translation asset folder + auth scaffolding
  Phase 3: Analysis → Integrate toggles, update header, validate login/register pages
  Phase 4: Synthesis → Full end-to-end testing + cleanup + deployment readiness
```

## RESPONSE_TEXT

Successfully completed the implementation planning for Authentication, Personalization, and Translation features in the static Docusaurus site. Created comprehensive documentation artifacts:

1. Implementation Plan (plan.md): Architecture overview and project structure
2. Research Findings (research.md): Technical decisions and rationale
3. Data Model (data-model.md): Entity definitions and validation rules
4. Quickstart Guide (quickstart.md): Implementation steps and code examples
5. API Contracts (contracts/api-contracts.md): Interface specifications

All key decisions were documented:
- Better-Auth client-only strategy for static hosting compatibility
- Static JSON storage for Urdu translations
- React Context with Docusaurus Root.js for global state management
- Conditional header rendering based on auth state

The plan addresses all technical requirements including SSR safety, GitHub Pages deployment constraints, and Better-Auth integration patterns.

## OUTCOME

The planning phase is complete with all required artifacts created. The implementation plan provides clear guidance for developing the Authentication, Personalization, and Translation features while maintaining static hosting compatibility and SSR safety.

## EVALUATION

The planning successfully captures all architectural decisions and provides clear implementation guidance. All user requirements from the prompt have been addressed with appropriate technical solutions that align with Better-Auth documentation and Docusaurus best practices.