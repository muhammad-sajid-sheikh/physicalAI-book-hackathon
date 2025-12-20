# Data Model: Docusaurus UI Upgrade

**Feature**: Docusaurus UI Upgrade | **Branch**: `001-docusaurus-ui-upgrade` | **Date**: 2025-12-21
**Purpose**: Document the structure of configuration files and data entities for the UI upgrade

## Overview

This document describes the data models and configuration structures for the physicalAI-book Docusaurus UI upgrade. It covers the main configuration files that will be modified to achieve the desired UI improvements while maintaining compatibility with existing content and navigation.

## Core Configuration Files

### 1. docusaurus.config.js

The main configuration file that controls the overall site behavior, theme settings, and plugin configurations. Based on the existing configuration in the physicalAI-book project.

**Structure**:
- `title`: Site title displayed in browser tab and navbar ("Physical AI & Humanoid Robotics")
- `tagline`: Brief description of the site ("An AI-Native Textbook")
- `favicon`: Path to favicon file ("img/favicon.ico")
- `url`: Base URL for the site ("https://your-docusaurus-site.example.com")
- `baseUrl`: Base URL relative path ("/")
- `organizationName`: GitHub organization/username ("facebook")
- `projectName`: GitHub repository name ("docusaurus")
- `future`: Future compatibility flags object (v4: true)
- `onBrokenLinks`: How to handle broken links ("throw")
- `i18n`: Internationalization configuration (defaultLocale: 'en', locales: ['en'])
- `presets`: Docusaurus presets (classic with docs, blog, and theme options)
- `themeConfig`: Theme-specific configuration object containing navbar, footer, colorMode, and prism settings

**Preset Configuration**:
- `docs`: Documentation plugin with sidebarPath and editUrl
- `blog`: Blog plugin with reading time, feed options, and editUrl
- `theme`: Theme plugin with customCss path ("./src/css/custom.css")

**Theme Configuration**:
- `colorMode`: Color mode settings (respectPrefersColorScheme: true)
- `navbar`: Navigation bar configuration with title, logo, and items
- `footer`: Footer configuration with links and copyright
- `prism`: Code block syntax highlighting theme

### 2. sidebars.js

Navigation configuration that defines the sidebar structure and organization of documentation content. Based on the existing configuration in the physicalAI-book project.

**Structure**:
- `tutorialSidebar`: Main sidebar array containing navigation items
- `type`: Item type ('doc', 'category', 'link', etc.)
- `label`: Display text for the navigation item
- `id`: Document identifier for linking
- `items`: Array of child items for categories
- `collapsed`: Boolean indicating initial collapsed state (default: true if not specified)
- `collapsible`: Boolean indicating if the category can be collapsed (default: true if not specified)

**Current Navigation Structure**:
- `'intro'`: Introduction document at the top level
- `'introduction'`: Document under 'Introduction to Physical AI' category
- Module 1: 'Module 1 - Robotic Nervous System (ROS 2)' with 4 items (index, chapter-1-1, chapter-1-2, chapter-1-3)
- Module 2: 'Module 2 - The Digital Twin (Gazebo & Unity)' with 4 items (index, chapter-2-1, chapter-2-2, chapter-2-3)
- Module 3: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac™)' with 4 items (index, chapter-3-1, chapter-3-2, chapter-3-3)
- Module 4: 'Module 4 - Vision-Language-Action (VLA)' with 4 items (index, voice-to-action, cognitive-planning, autonomous-capstone)

**UI Enhancement Opportunities**:
- Add `collapsed: false` to keep main module categories expanded by default for better visibility
- Add `collapsible: true` explicitly to maintain consistency
- Consider adding links to category headers using the `link` property
- Organize sub-chapters within modules using nested categories for complex modules

### 3. src/css/custom.css

Custom CSS file that overrides default Docusaurus styling with project-specific UI enhancements. Based on the existing configuration in the physicalAI-book project.

**Structure**:
- CSS variables following Infima naming convention
- Custom class definitions
- Responsive media queries
- Component-specific styling overrides

**Current CSS Variables**:
- `--ifm-color-primary`: Primary color (#2e8555 in light mode, #25c2a0 in dark mode)
- `--ifm-color-primary-dark`: Darker shade of primary color
- `--ifm-color-primary-darker`: Even darker shade of primary color
- `--ifm-color-primary-darkest`: Darkest shade of primary color
- `--ifm-color-primary-light`: Lighter shade of primary color
- `--ifm-color-primary-lighter`: Even lighter shade of primary color
- `--ifm-color-primary-lightest`: Lightest shade of primary color
- `--ifm-code-font-size`: Code font size (95%)
- `--docusaurus-highlighted-code-line-bg`: Background for highlighted code lines

**UI Enhancement Opportunities**:
- Expand color palette with additional theme variables for better visual hierarchy
- Add typography variables for improved readability (font families, line heights, spacing)
- Implement responsive spacing variables for better mobile experience
- Add custom variables for module-specific styling
- Enhance code block styling with improved syntax highlighting
- Add accessibility-focused variables for better contrast ratios
- Implement custom component styling for educational content elements

## Data Entities and Relationships

### Site Configuration Entity
- **Properties**: title, tagline, URL, base URL, organization details
- **Relationships**: Connected to all other configuration entities
- **Purpose**: Defines the fundamental site properties

### Theme Configuration Entity
- **Properties**: navbar, footer, color mode, prism, metadata
- **Relationships**: Depends on Site Configuration
- **Purpose**: Controls the visual appearance and theming

### Navigation Configuration Entity
- **Properties**: sidebar structure, navbar items, breadcrumbs
- **Relationships**: Connected to documentation content structure
- **Purpose**: Defines user navigation paths through content

### Styling Configuration Entity
- **Properties**: CSS variables, custom styles, responsive breakpoints
- **Relationships**: Applied across all site pages and components
- **Purpose**: Controls visual presentation and UI enhancements

## Configuration Schema Details

### Theme Configuration Schema

```javascript
themeConfig: {
  navbar: {
    title: string,
    logo: {
      alt: string,
      src: string,
    },
    items: [
      {
        type: 'doc' | 'docSidebar' | 'dropdown' | 'search' | 'html',
        position: 'left' | 'right',
        label?: string,
        to?: string,
        docId?: string,
        sidebarId?: string,
        items?: [ /* nested items */ ]
      }
    ]
  },
  footer: {
    style: 'dark' | 'light',
    links: [
      {
        title: string,
        items: [
          {
            label: string,
            to?: string,
            href?: string
          }
        ]
      }
    ],
    copyright: string
  },
  colorMode: {
    defaultMode: 'light' | 'dark',
    disableSwitch: boolean,
    respectPrefersColorScheme: boolean
  },
  prism: {
    theme: object,
    darkTheme: object,
    defaultLanguage: string,
    additionalLanguages: [string]
  }
}
```

### Sidebar Configuration Schema

```javascript
{
  type: 'category' | 'doc' | 'link',
  label: string,  // Required for category and link types
  id: string,     // Required for doc type
  href: string,   // Required for link type
  items: [ /* array of child items */ ],
  collapsed: boolean,
  collapsible: boolean,
  link: {
    type: 'doc' | 'generated-index',
    id: string
  }
}
```

## UI Enhancement Data Structure

### Color System
- **Primary Colors**: 7 shades (darkest to lightest) for proper contrast ratios
- **Secondary Colors**: Supporting color palette
- **Functional Colors**: Success, warning, error states
- **Theme Colors**: Light and dark mode variants

### Typography System
- **Font Family**: Base font stack with fallbacks
- **Font Sizes**: Responsive scaling system
- **Line Heights**: Readability-optimized values
- **Font Weights**: Hierarchy-establishing weights

### Spacing System
- **Base Unit**: Consistent spacing measurement
- **Scale**: Proportional spacing values
- **Responsive Values**: Mobile and desktop variations

## Implementation Considerations

### Backward Compatibility
- All existing document IDs must remain unchanged
- Navigation paths must preserve existing URLs
- Content structure must remain intact

### Performance
- CSS bundle size should be optimized
- Asset loading should be efficient
- Critical rendering path should be preserved

### Accessibility
- Color contrast ratios must meet WCAG 2.1 AA standards
- Navigation must be keyboard accessible
- Semantic HTML structure must be maintained

This data model provides the foundation for implementing the UI upgrade while preserving all existing content and functionality.