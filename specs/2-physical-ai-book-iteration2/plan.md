# Implementation Plan: Physical AI Book (Iteration 2)

**Branch**: `2-physical-ai-book-iteration2` | **Date**: 2025-12-08 | **Spec**: [link to spec.md](spec.md)

**Input**: Feature specification from `/specs/2-physical-ai-book-iteration2/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Transform the completed high-level Physical AI book structure into a fully implementable Docusaurus project hosted on GitHub Pages with authentication, personalization, Urdu translation, and MCP server automation. The implementation will restructure the existing Docusaurus template, implement better-auth for user management, add personalization features based on user profiles, implement Urdu translation capabilities, and integrate MCP servers for automated content generation.

## Technical Context

**Language/Version**: JavaScript/TypeScript, Node.js LTS
**Primary Dependencies**: Docusaurus, React, better-auth, better-sqlite3, Context7 MCP servers
**Storage**: GitHub Pages (static hosting), better-sqlite3 for local author environment only
**Testing**: Jest for unit tests, Cypress for E2E tests
**Target Platform**: Web browser, GitHub Pages
**Project Type**: Web/documentation site
**Performance Goals**: Pages load in under 3 seconds, support minimal scale (dozens of users)
**Constraints**: Static hosting only (no backend), better-auth in static mode, Urdu translation must be pre-generated
**Scale/Scope**: Minimal scale (dozens of users) for a small, focused audience

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution and requirements:
- ✅ Static hosting compliance: All features must work with static hosting (GitHub Pages)
- ✅ Authentication: better-auth in static mode only (no backend)
- ✅ Translation: Pre-generated Urdu content (no runtime translation API)
- ✅ MCP integration: Automated content generation and validation
- ✅ Performance: Under 3 seconds page load time
- ✅ Scale: Designed for minimal user scale (dozens of users)

## Project Structure

### Documentation (this feature)

```text
specs/2-physical-ai-book-iteration2/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus Documentation Site Structure
docs/
├── intro.md
├── module-1-ros2/
├── module-2-simulation/
├── module-3-isaac/
├── module-4-vla/
├── capstone/
├── hardware-lab/
└── conclusion/

src/
├── components/          # Custom React components (auth, personalization, translation)
│   ├── Auth/
│   ├── Personalization/
│   └── Translation/
├── pages/               # Static pages
├── theme/               # Custom theme components
│   └── MDXComponents/
├── css/                 # Custom styles
└── utils/               # Utility functions

static/
├── img/                 # Static images
└── urdu/                # Pre-generated Urdu MDX files

docusaurus.config.js      # Docusaurus configuration
sidebars.js              # Navigation structure
package.json             # Dependencies (docusaurus, better-auth, etc.)
```

**Structure Decision**: Single Docusaurus documentation site with custom components for authentication, personalization, and translation. The structure leverages Docusaurus' built-in documentation features while adding custom functionality through React components and theme overrides.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [No violations identified] | [All requirements comply with constitution] |