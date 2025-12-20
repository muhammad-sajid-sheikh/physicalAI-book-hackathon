# Quickstart Guide: Vision-Language-Action (VLA) Module

## Overview
This guide provides the essential steps to create and integrate the Vision-Language-Action (VLA) module into the Docusaurus-based textbook.

## Prerequisites
- Node.js 18+ installed
- Basic understanding of Docusaurus documentation structure
- Knowledge of Markdown syntax
- Access to the project repository

## Step-by-Step Setup

### 1. Create the Module Directory
```bash
mkdir -p docs/module-04
```

### 2. Create the Chapter Files
Create three markdown files in the `docs/module-04/` directory:
- `voice-to-action.md` - Voice-to-Action processing
- `cognitive-planning.md` - Cognitive Planning with LLMs
- `autonomous-capstone.md` - Autonomous Humanoid Capstone

### 3. Add Module Introduction
Create `docs/module-04/index.md` as the entry point for the module.

### 4. Update Navigation
Update the sidebar configuration to include the new module:
- Add the module to `sidebars.js` or docusaurus config
- Ensure proper ordering with other modules
- Include all three chapters in the navigation

### 5. Content Guidelines
When writing the chapters:
- Follow the constitution requirements for citations (20+ sources, 40% peer-reviewed)
- Include runnable code examples and snippets
- Maintain accessibility standards
- Provide "Sources & Further Reading" sections

## Testing the Documentation
1. Run the local Docusaurus server: `npm run start`
2. Navigate to the new module pages
3. Verify all links work correctly
4. Check that the sidebar navigation includes the new module
5. Validate that all examples render properly

## Validation Checklist
- [ ] Module appears in sidebar navigation
- [ ] All three chapters are accessible
- [ ] Cross-references between chapters work
- [ ] Code examples render correctly
- [ ] Local build completes without errors
- [ ] All external links are valid