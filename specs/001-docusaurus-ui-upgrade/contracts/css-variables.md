# CSS Variable Contracts: Docusaurus UI Upgrade

**Feature**: Docusaurus UI Upgrade | **Branch**: `001-docusaurus-ui-upgrade` | **Date**: 2025-12-21
**Purpose**: Define contracts for CSS variables used in the UI upgrade

## Overview

This document defines the contracts for CSS variables that will be used to implement the UI enhancements in the physicalAI-book Docusaurus project. These contracts ensure consistent theming, maintainability, and proper integration with the Docusaurus framework.

## Required CSS Variables

### Primary Color System (Required)
The primary color system requires 7 shades to ensure proper contrast ratios and visual consistency:

```css
:root {
  --ifm-color-primary: #2e8555;          /* Base primary color */
  --ifm-color-primary-dark: #29784c;     /* Darker shade */
  --ifm-color-primary-darker: #277148;   /* Even darker shade */
  --ifm-color-primary-darkest: #205d3b;  /* Darkest shade */
  --ifm-color-primary-light: #33925d;    /* Lighter shade */
  --ifm-color-primary-lighter: #359962;  /* Even lighter shade */
  --ifm-color-primary-lightest: #3cad6e; /* Lightest shade */
}
```

**Contract Requirements**:
- All 7 shades must be defined for proper contrast ratios
- Colors must maintain WCAG 2.1 AA compliance
- Dark mode variants must be provided
- Each shade should represent a specific contrast level for different UI elements

### Dark Mode Color System (Required)
```css
[data-theme='dark'] {
  --ifm-color-primary: #25c2a0;          /* Primary in dark mode */
  --ifm-color-primary-dark: #21af90;     /* Darker shade in dark mode */
  --ifm-color-primary-darker: #1fa588;   /* Even darker in dark mode */
  --ifm-color-primary-darkest: #1a8870;  /* Darkest in dark mode */
  --ifm-color-primary-light: #29d5b0;    /* Lighter shade in dark mode */
  --ifm-color-primary-lighter: #32d8b4;  /* Even lighter in dark mode */
  --ifm-color-primary-lightest: #4fddbf; /* Lightest in dark mode */
}
```

**Contract Requirements**:
- Dark mode variables must be defined for all primary colors
- Contrast ratios must remain WCAG 2.1 AA compliant
- Color relationships should be maintained between light and dark modes

## Typography Variables (Required)

### Font System
```css
:root {
  --ifm-font-family-base: system-ui, -apple-system, sans-serif;
  --ifm-font-size-base: 16px;
  --ifm-line-height-base: 1.6;
  --ifm-code-font-size: 95%;
}
```

**Contract Requirements**:
- Base font size must be between 16-18px for optimal readability
- Line height must be between 1.5-1.6 for readability
- Code font size must be smaller than base font for distinction

## Spacing Variables (Required)

### Layout Spacing
```css
:root {
  --ifm-spacing-horizontal: 1.5rem;
  --ifm-spacing-vertical: 1rem;
}
```

**Contract Requirements**:
- Horizontal spacing must provide adequate content margins
- Vertical spacing must ensure proper content separation
- Values must work across all screen sizes

## Component-Specific Variables (Required)

### Code Block Styling
```css
:root {
  --docusaurus-highlighted-code-line-bg: rgba(0, 0, 0, 0.1);
}
```

**Contract Requirements**:
- Highlighted code line background must be subtle but visible
- Must work in both light and dark modes
- Opacity should be between 0.1-0.3 for appropriate contrast

## Optional CSS Variables (Enhancement)

### Container Width
```css
:root {
  --ifm-container-width: 1200px;
  --ifm-container-width-xl: 1400px;
}
```

**Contract Requirements**:
- Container width must not exceed comfortable reading width (75 characters max)
- XL width should be used for wide content displays
- Must maintain proper content margins

### Breadcrumb Styling
```css
:root {
  --ifm-breadcrumb-spacing: 0.5rem;
  --ifm-breadcrumb-color-active: var(--ifm-color-primary);
}
```

**Contract Requirements**:
- Breadcrumb spacing must be consistent with other navigation elements
- Active breadcrumb color should use primary color for consistency

### Alert and Note Components
```css
:root {
  --ifm-alert-background-color: var(--ifm-background-surface-color);
  --ifm-alert-border-left-width: 0.25rem;
}
```

**Contract Requirements**:
- Alert backgrounds should maintain good contrast
- Border width should be visible but not overwhelming

## Responsive CSS Variables

### Breakpoint Variables
While Docusaurus doesn't use CSS variables for breakpoints directly, custom responsive styles may use:

```css
/* These are used in media queries rather than direct variables */
@media (max-width: 996px) {
  :root {
    --ifm-spacing-horizontal: 1rem;
    --ifm-spacing-vertical: 0.75rem;
  }
}
```

**Contract Requirements**:
- Mobile spacing should be more compact than desktop
- Typography may need adjustment for smaller screens
- Touch targets should remain accessible

## Accessibility Contracts

### Color Contrast Requirements
All color variables must meet:
- 4.5:1 contrast ratio for normal text
- 3:1 contrast ratio for large text
- Sufficient contrast for UI elements and interactive components

### High Contrast Mode Considerations
```css
@media (prefers-contrast: high) {
  :root {
    --ifm-color-primary: #000000;  /* High contrast option */
  }
}
```

## Performance Considerations

### Variable Optimization
- Minimize the number of custom CSS variables
- Reuse variables where possible instead of creating new ones
- Group related variables logically

### Bundle Size Impact
- CSS variable definitions have minimal impact on bundle size
- Focus on optimizing CSS selectors and reducing redundancy
- Use variables consistently to avoid duplicate values

## Integration Contracts

### Docusaurus Framework Compatibility
- CSS variables must use the `--ifm-` prefix convention
- Variables should extend rather than replace Docusaurus defaults
- Custom variables should follow the same naming pattern

### Update Compatibility
- CSS variable approach maintains compatibility with Docusaurus updates
- Variables should be defined in `src/css/custom.css`
- Avoid modifying Docusaurus core files

## Validation Criteria

### Compliance Checks
- [ ] All required primary color variables are defined
- [ ] Dark mode variables are properly implemented
- [ ] Typography variables meet readability standards
- [ ] Spacing variables provide adequate content separation
- [ ] Color contrast ratios meet WCAG 2.1 AA standards
- [ ] Variables follow Docusaurus naming conventions
- [ ] Performance impact is minimized

### Testing Requirements
- [ ] Color contrast validated using automated tools
- [ ] Responsive behavior tested across device sizes
- [ ] Accessibility features verified with screen readers
- [ ] Cross-browser compatibility confirmed

These contracts ensure that CSS variable usage remains consistent, accessible, and maintainable throughout the UI upgrade process.