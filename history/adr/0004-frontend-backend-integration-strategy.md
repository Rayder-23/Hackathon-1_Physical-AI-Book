# ADR 0004: Frontend-Backend Integration Strategy

## Status
Accepted

## Date
2025-12-18

## Context
The Physical AI Book platform requires tight integration between the Docusaurus frontend and FastAPI backend for authentication functionality. We need to determine how authentication state flows between these components, especially given static hosting constraints. The existing codebase has some authentication infrastructure that needs to be integrated with the new better-auth system.

Key constraints include:
- Static hosting on GitHub Pages
- Existing Docusaurus architecture
- Integration with existing RAG backend
- Need to maintain existing functionality while adding new features

## Decision
We will use an update strategy that integrates with existing auth-related files rather than replacing them. The approach includes:

- Frontend: Docusaurus React components using better-auth hooks integrated into existing AuthContext
- Backend: FastAPI endpoints extending the existing backend to support better-auth
- Integration: API calls from frontend to backend for authentication operations
- State management: Context providers for authentication state in Docusaurus

The authentication boundary will be managed through better-auth's server-side session handling with React components that communicate with the backend API.

## Alternatives Considered
- Complete replacement of existing auth infrastructure: Rejected due to risk of breaking existing functionality
- Parallel implementation: Rejected due to maintenance overhead and complexity
- Standalone authentication service: Rejected due to complexity and integration challenges
- Client-side only approach: Rejected due to security concerns with static hosting

## Consequences
### Positive
- Maintains existing functionality while adding new features
- Minimizes disruption to existing codebase
- Leverages existing architecture patterns
- Maintains compatibility with Docusaurus framework
- Secure server-side session management

### Negative
- More complex integration than fresh implementation
- Potential for technical debt from mixed approaches
- Requires careful coordination between components
- May require refactoring of existing auth code

## References
- specs/4-auth-profiling-integration/plan.md
- specs/4-auth-profiling-integration/research.md
- specs/4-auth-profiling-integration/quickstart.md