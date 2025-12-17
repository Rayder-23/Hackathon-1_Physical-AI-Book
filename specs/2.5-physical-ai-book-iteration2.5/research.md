# Research: Authentication, Personalization, and Translation Implementation

## Decision: Better-Auth Client-Only Strategy
**Rationale**: Selected Better-Auth client-only approach for static hosting compatibility with GitHub Pages. This approach allows for proper authentication flow while maintaining static site constraints. Server-side auth would violate static hosting requirements.

**Alternatives considered**:
- Remote auth API: Would require server infrastructure, violating static site constraints
- Custom auth solution: Would require more development time and maintenance

## Decision: Static JSON Translation Storage
**Rationale**: Selected static JSON under /static/i18n for fast loading and easy toggle functionality. This approach provides instant translation switching without network requests.

**Alternatives considered**:
- MDX duplication for Urdu: Would create maintenance overhead and content duplication
- Runtime translation API: Would require network requests and add complexity

## Decision: React Context via Docusaurus Root.js Wrapper
**Rationale**: Selected React Context pattern with Docusaurus Root.js wrapper for global state availability across all pages. This ensures consistent UX and state persistence.

**Alternatives considered**:
- Per-page local state: Would create inconsistent UX and require state synchronization
- Redux/store: Would add unnecessary complexity for this use case

## Decision: Conditional Header Rendering
**Rationale**: Selected conditional rendering based on auth state for intuitive UI. This provides immediate visual feedback to users about their authentication status.

**Alternatives considered**:
- Separate pages for toggle actions: Would require more navigation and reduce usability

## Technical Research Findings

### Better-Auth Client Integration
- Better-Auth supports client-only patterns with `isAuthenticated` checks
- Session persistence through localStorage works in static environments
- Client-side validation and token management are sufficient for this use case

### Docusaurus SSR-Safe Patterns
- Use `typeof window !== 'undefined'` checks before accessing browser APIs
- Implement proper context provider wrapping in Root.js
- Ensure all hooks are used within appropriate provider contexts

### Translation Asset Pipeline
- Static JSON files can be loaded via dynamic imports
- Translation switching can be implemented with React state management
- Urdu content should be pre-translated and stored in structured format

### GitHub Pages Deployment Constraints
- No server-side rendering dependencies allowed
- All functionality must be client-side
- Build process must complete without errors