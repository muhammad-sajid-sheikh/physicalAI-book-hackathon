# Quickstart Guide: Docusaurus UI Upgrade

**Feature**: Docusaurus UI Upgrade | **Branch**: `001-docusaurus-ui-upgrade` | **Date**: 2025-12-21
**Purpose**: Quick implementation guide for UI enhancements to the physicalAI-book Docusaurus project

## Overview

This quickstart guide provides the essential steps to implement UI enhancements for the physicalAI-book Docusaurus project. Follow these steps to upgrade the visual design, navigation, and readability while maintaining all existing content.

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Access to the physicalAI-book Docusaurus project files
- Basic understanding of CSS and Docusaurus configuration

## Quick Setup Steps

### 1. Clone and Prepare the Project

```bash
cd physicalAI-book
npm install  # or yarn install
```

### 2. Start the Development Server

```bash
npm run start  # or yarn start
```

This will launch the development server at `http://localhost:3000` where you can see your changes in real-time.

## CSS Customization Quick Guide

### 1. Update Color Scheme

Modify the CSS variables in `src/css/custom.css` to update the color palette:

```css
:root {
  /* Replace the default green color scheme with your preferred colors */
  --ifm-color-primary: #1a73e8;          /* Primary blue */
  --ifm-color-primary-dark: #1865d0;     /* Darker blue */
  --ifm-color-primary-darker: #175db9;   /* Even darker blue */
  --ifm-color-primary-darkest: #134c99;  /* Darkest blue */
  --ifm-color-primary-light: #2a80eb;    /* Light blue */
  --ifm-color-primary-lighter: #3a8bec;  /* Lighter blue */
  --ifm-color-primary-lightest: #5b9dec; /* Lightest blue */
}
```

### 2. Enhance Typography

Add typography improvements to `src/css/custom.css`:

```css
:root {
  /* Use a modern font stack */
  --ifm-font-family-base: 'Inter', system-ui, -apple-system, sans-serif;
  --ifm-line-height-base: 1.6;
  --ifm-font-size-base: 17px;  /* Slightly larger base font for readability */
  --ifm-code-font-size: 90%;
}
```

### 3. Improve Spacing and Layout

Add spacing improvements to `src/css/custom.css`:

```css
:root {
  --ifm-spacing-horizontal: 1.5rem;
  --ifm-spacing-vertical: 1.5rem;
}
```

## Navigation Enhancement Quick Steps

### 1. Update Sidebar for Better Module Organization

Edit `sidebars.js` to enhance module organization:

```javascript
{
  type: 'category',
  label: 'Module 1 - Robotic Nervous System (ROS 2)',
  collapsed: false,  // Keep expanded by default for better UX
  collapsible: true,
  link: {
    type: 'doc',
    id: 'module-1/index',  // Link to the module's main page
  },
  items: [
    'module-1/chapter-1-1',
    'module-1/chapter-1-2',
    'module-1/chapter-1-3',
  ],
}
```

### 2. Enhance Navbar Structure

Update `docusaurus.config.js` to improve navigation:

```javascript
// In themeConfig.navbar.items
{
  type: 'dropdown',
  label: 'Modules',
  position: 'left',
  items: [
    {
      label: 'Module 1 - Robotic Nervous System (ROS 2)',
      to: '/docs/module-1/index',
    },
    {
      label: 'Module 2 - The Digital Twin (Gazebo & Unity)',
      to: '/docs/module-2/index',
    },
    {
      label: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac™)',
      to: '/docs/module-3/index',
    },
    {
      label: 'Module 4 - Vision-Language-Action (VLA)',
      to: '/docs/module-04/index',
    },
  ],
}
```

## Responsive Design Enhancements

### 1. Add Mobile-First Improvements

Add responsive styles to `src/css/custom.css`:

```css
/* Improve mobile navigation */
@media screen and (max-width: 996px) {
  .navbar-sidebar {
    background-color: var(--ifm-background-surface-color);
  }

  /* Ensure adequate touch targets */
  .menu__list-item {
    margin: 0.25rem 0;
  }
}
```

## Accessibility Improvements

### 1. Ensure Color Contrast Compliance

Verify that your color choices meet WCAG 2.1 AA standards (4.5:1 for normal text, 3:1 for large text).

### 2. Add Focus Indicators

Docusaurus handles focus indicators automatically, but you can customize them if needed:

```css
:root {
  --ifm-color-primary: #1a73e8;
  --ifm-color-primary-contrast-background: #ffffff;
  --ifm-color-primary-contrast-foreground: #000000;
}
```

## Testing Your Changes

### 1. Check Different Screen Sizes

- Use browser developer tools to test mobile, tablet, and desktop views
- Verify that navigation remains accessible on all screen sizes
- Ensure text remains readable across devices

### 2. Verify All Modules Are Accessible

- Navigate through each module in the sidebar
- Check that all links work correctly
- Verify that code blocks display properly

## Common Customization Examples

### 1. Custom Homepage Banner

Create a more engaging homepage by customizing the index page styles in `src/css/custom.css`:

```css
.hero_banner {
  padding: 4rem 0;
  text-align: center;
  position: relative;
  overflow: hidden;
  background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%);
}

@media screen and (max-width: 996px) {
  .hero_banner {
    padding: 2rem;
  }
}
```

### 2. Module-Specific Styling

Add visual distinction for different modules:

```css
/* Add a left border to distinguish content sections */
.markdown > h2 {
  border-left: 4px solid var(--ifm-color-primary);
  padding-left: 1rem;
  margin-top: 2rem;
}
```

## Deployment

When ready to deploy your changes:

1. Build the site: `npm run build`
2. Test locally: `npm run serve`
3. Deploy to your hosting platform (GitHub Pages, etc.)

## Next Steps

After completing these quick setup steps:
1. Review the detailed recommendations in the research.md file
2. Consider implementing additional accessibility features
3. Test with real users to validate the improved navigation and readability
4. Optimize performance by reviewing CSS bundle size