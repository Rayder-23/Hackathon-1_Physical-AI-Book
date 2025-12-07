# Research: Physical AI Book (Iteration 2)

**Date**: 2025-12-08
**Feature**: Physical AI Book (Iteration 2)
**Research completed**: Yes

## Docusaurus Architecture Research

### Decision: Docusaurus file restructuring
**Rationale**: The existing Docusaurus template needs restructuring to match the Physical AI book requirements while maintaining Docusaurus best practices.
**Alternatives considered**:
- Keep blog: Not needed for book structure
- Remove blog: Appropriate since this is a book, not a blog
- Keep other default content: Will be replaced with book content

**Chosen approach**: Remove blog section, restructure docs/ to follow the 4-module book structure (ROS 2, Simulation, Isaac AI, VLA)

### Decision: MDX conventions
**Rationale**: MDX files need to follow Docusaurus conventions for proper integration with the documentation system.
**Chosen approach**: Use standard Docusaurus MDX format with frontmatter for metadata, consistent heading hierarchy, and proper linking conventions.

## Authentication Model Research

### Decision: Static-only better-auth implementation
**Rationale**: The requirement specifies that better-auth must operate entirely in static-hosted mode (no backend), which aligns with GitHub Pages constraints.
**Alternatives considered**:
- Full backend authentication: Violates static hosting constraint
- Alternative auth providers: Would add complexity without benefit
- Client-side only auth: Insufficient for user management

**Chosen approach**: Implement better-auth in static mode with client-side guards and local storage for tokens.

### Decision: Token storage strategy
**Rationale**: Tokens must persist client-side only for logged users as per constraints.
**Chosen approach**: Store authentication tokens in browser's localStorage with proper security considerations.

### Decision: Client-side guards
**Rationale**: Need to protect author-only content without backend validation.
**Chosen approach**: Implement React-based route guards that check authentication status before rendering protected content.

## Data Handling for Personalization Research

### Decision: User background storage location
**Rationale**: User software/hardware background information needs to be stored for personalization.
**Alternatives considered**:
- Server-side storage: Not possible with static hosting
- Local storage: Appropriate for static site personalization
- Session storage: Would reset on page reload

**Chosen approach**: Store user background information in browser's localStorage, associated with authenticated user profile.

### Decision: MDX content adaptation
**Rationale**: Personalization needs to modify chapter content based on user profile.
**Chosen approach**: Use React components within MDX to conditionally render content based on user preferences stored in localStorage.

## Translation Design Research

### Decision: Pre-generated Urdu MDX implementation
**Rationale**: The requirement specifies that Urdu translation must be static (no runtime translation API).
**Alternatives considered**:
- Runtime translation: Violates static hosting constraint
- Pre-generated files: Requires file management but works with static hosting
- Hybrid approach: Would add complexity

**Chosen approach**: Pre-generate Urdu MDX files with corresponding English file structure, implement toggle to switch between languages.

## GitHub Pages Deployment Research

### Decision: Deployment path and branch strategy
**Rationale**: Need to determine how to deploy to GitHub Pages with proper URL structure.
**Chosen approach**: Deploy to root domain using gh-pages branch with proper baseUrl configuration in docusaurus.config.js.

## MCP Usage Strategy Research

### Decision: Automation approach
**Rationale**: Need to determine the best way to integrate MCP servers for automated content generation.
**Alternatives considered**:
- Auto-scaffold: More efficient but potentially less flexible
- Manual creation: More control but more work
- Directory extraction: Good for maintaining consistency

**Chosen approach**: Use MCP automation for scaffolding, sidebar generation, and translation validation to maintain consistency and reduce manual work.

## Technical Integration Research

### Docusaurus + better-auth Integration
**Research findings**: better-auth can work in static mode with Docusaurus using custom React components for login/logout functionality and route protection.

### Personalization Implementation
**Research findings**: Docusaurus supports custom MDX components that can access browser storage to provide personalized content experiences.

### Translation Toggle Implementation
**Research findings**: Language switching can be implemented using React state management and file-based routing, with pre-generated translation files.

## Quality Validation Plan

### Build Integrity
- Docusaurus build must pass without warnings
- All links and sidebar entries must resolve correctly
- MDX syntax validation using MCP servers

### Deployment Reliability
- Site must load at correct URL with proper baseUrl configuration
- All assets must load correctly on GitHub Pages
- Cross-browser compatibility testing

### Authentication Correctness
- Protected routes require login
- Signup flow properly captures and stores user background
- Tokens persist only client-side for logged users

### Personalization Validation
- Personalized content modifies based on user profile
- Fallback content displays when personalization is disabled
- User preferences persist across sessions

### Translation Validation
- Urdu translation toggle loads content without breaking navigation
- All Urdu files pass MDX syntax validation
- Translation quality and accuracy verification

### MCP Tooling Stability
- Scaffolding produces correct directory structure
- Sidebar auto-generation produces valid sidebars.js
- Content generation yields compilable MDX files