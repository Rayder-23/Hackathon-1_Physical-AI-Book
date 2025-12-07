<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles:
- Technical Accuracy (was PRINCIPLE_1_NAME)
- Clarity (was PRINCIPLE_2_NAME)
- Consistency (was PRINCIPLE_3_NAME)
- Reliability (was PRINCIPLE_4_NAME)
- Documentation Style (was PRINCIPLE_5_NAME)
Added sections: Content Standards, Writing Requirements, Code Standards, Structure & Formatting
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ⚠ pending review
Follow-up TODOs: None
-->

# Humanoid Robotics Book Constitution

## Core Principles

### Technical Accuracy
All explanations must be correct and based on authoritative sources. No hallucinated tools, APIs, or concepts are allowed. All content must be verified against official documentation, standards, and reputable technical references.

### Clarity
Content should be easy to understand for students, developers, and tech learners. Language must be simple, direct, educational, and free of unnecessary jargon. Each concept should be explained in an accessible manner without sacrificing technical precision.

### Consistency
Tone, structure, formatting, and terminology must remain uniform across chapters. All chapters must follow a consistent template with Overview, Key Concepts, Step-by-step Explanations, Code Examples, Practical Tips, Summary, and References sections.

### Reliability
All code examples must run successfully or pass tool-based validation. No fictional libraries, tools, or commands are allowed. All code must be valid and reproducible through Claude Code testing or manual verification.

### Documentation Style
Write with the precision and structure of high-quality technical documentation. Content must follow established documentation patterns with clear headings, proper formatting, and professional presentation suitable for educational purposes.

### Content Authenticity
Zero hallucinations are permitted. No unverifiable or speculative claims are allowed. No proprietary content unless publicly documented. All sources must be cited in Markdown with a reference list at the end of each chapter.

## Content Standards

### Source Requirements:
- Use official documentation, standards, and reputable technical references
- Cite sources in Markdown with a reference list at the end of each chapter
- All claims must be verifiable through authoritative sources

### Writing Requirements:
- Each chapter must follow a consistent template:
  1. Overview
  2. Key Concepts
  3. Step-by-step Explanations
  4. Code Examples (tested)
  5. Practical Tips
  6. Summary
  7. References
- Language: simple, direct, educational, and free of unnecessary jargon
- Minimum 8 main chapters + Introduction + Conclusion
- Each chapter: ~1,000–2,500 words

### Code Standards:
- All code examples must run successfully or be tested via Claude Code
- Code should be formatted for MDX compatibility inside Docusaurus
- No fictional libraries, tools, or commands
- Code examples must be validated to ensure they execute correctly

### Structure & Formatting:
- Use modular sections that fit naturally into the Docusaurus `/docs` folder
- Follow consistent filenames, slugs, and headings
- Maintain compatibility with Docusaurus build system
- Ensure proper navigation and linking between chapters

## Development Workflow

### Quality Gates:
- Docusaurus site must build successfully with no warnings or broken links
- All code examples must execute correctly or pass tool-based validation
- Content must pass plagiarism and authenticity checks
- Each chapter must include proper citations and references

### Review Process:
- Technical accuracy verification by subject matter experts
- Content clarity review by target audience representatives
- Code example validation through automated testing
- Cross-referencing and consistency checks across chapters

### Deployment Requirements:
- Book must compile successfully in Docusaurus
- Must deploy cleanly to GitHub Pages without errors
- All internal links must resolve correctly
- Search functionality must work across all content

## Governance

This constitution supersedes all other practices and guidelines for the Humanoid Robotics Book project. All project activities must comply with these principles.

All content creation and modification must verify compliance with technical accuracy, clarity, consistency, reliability, and documentation style requirements. Complexity must be justified with clear educational value.

Content authenticity must be maintained through rigorous source verification and citation practices. No proprietary or unverifiable information may be included without proper documentation.

All contributions must be reviewed for compliance with this constitution before acceptance.

**Version**: 1.1.0 | **Ratified**: 2025-01-01 | **Last Amended**: 2025-12-07