# Implementation Plan: Authentication, Personalization, and Translation Features

**Branch**: `iteration-2.5` | **Date**: 2025-12-10 | **Spec**: [link to spec.md](../2.5-physical-ai-book-iteration2.5/spec.md)
**Input**: Feature specification from `/specs/2.5-physical-ai-book-iteration2.5/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of Authentication, Personalization, and Translation features in a static Docusaurus site using Better-Auth client-only patterns. The solution includes user authentication with username/password, personalization settings for content difficulty and focus, and Urdu translation switching. All features are implemented with SSR-safe patterns for GitHub Pages compatibility.

## Technical Context

**Language/Version**: JavaScript/TypeScript, React 18+
**Primary Dependencies**: Better-Auth client, Docusaurus 3.x, React Context API
**Storage**: localStorage for session persistence, static JSON files for Urdu translations
**Testing**: Manual testing of auth workflows, SSR safety validation, translation toggle functionality
**Target Platform**: Static hosting (GitHub Pages), modern browsers
**Project Type**: Static web application
**Performance Goals**: Fast loading, no SSR errors, instant translation switching
**Constraints**: Static hosting compatible, SSR-safe, no server dependencies
**Scale/Scope**: Single Docusaurus site with authentication and translation features

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] All auth state client-side only (no server-side auth calls)
- [x] SSR-safe patterns with window environment checks
- [x] Static hosting compatibility (GitHub Pages)
- [x] Follows Docusaurus conventions (Root.js wrapper)
- [x] Better-Auth client-only patterns
- [x] No server-side rendering dependencies
- [x] Proper context provider architecture

## Project Structure

### Documentation (this feature)

```text
specs/1-auth-personalization-translation/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
src/
├── components/
│   ├── Auth/
│   │   ├── ProtectedRoute.jsx
│   │   └── [other auth components]
│   ├── Personalization/
│   │   └── Toggle.jsx
│   └── Translation/
│       └── Toggle.jsx
├── pages/
│   ├── login.jsx
│   └── register.jsx
├── theme/
│   ├── Root.js
│   └── MDXComponents/
│       ├── PersonalizedContent.jsx
│       ├── TranslatableContent.jsx
│       ├── ProtectedContent.jsx
│       ├── PersonalizedParagraph.jsx
│       ├── PersonalizedCode.jsx
│       └── PersonalizedSection.jsx
├── contexts/
│   ├── AuthContext.jsx
│   ├── PersonalizationContext.jsx
│   └── TranslationContext.jsx
├── hooks/
│   ├── useAuth.jsx
│   ├── usePersonalization.jsx
│   └── useTranslation.jsx
└── data/
    └── ur/
        └── book.json          # Static Urdu translations
```

**Structure Decision**: Single Docusaurus project with client-side authentication and translation features. All functionality is implemented within the existing project structure using Docusaurus conventions.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple context providers | Required for separate concerns | Combined context would create unnecessary complexity |
| Client-only auth | Required for static hosting compatibility | Server-dependent auth would break GitHub Pages deployment |