 // @ts-check

  // This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

  /**
   * Creating a sidebar enables you to:
   - create an ordered group of docs
   - render a sidebar for each doc of that group
   - provide next/previous navigation

   The sidebars can be generated from the filesystem, or explicitly defined here.

   Create as many sidebars as you want.

  //  @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
   */
  const sidebars = {
    // @ts-ignore
    tutorialSidebar: [
      'intro',
      //'hello', // <-- REMOVE this line or comment it out

      {
        type: 'category',
        label: 'Introduction to Physical AI',
        items: ['introduction'],
        collapsed: true,
        collapsible: true,
      },
      {
        type: 'category',
        label: 'Module 1 - Robotic Nervous System (ROS 2)',
        items: [
          'module-1/chapter-1-1',
          'module-1/chapter-1-2',
          'module-1/chapter-1-3',
        ],
        collapsed: false,  // Keep expanded by default for better UX
        collapsible: true,
        link: {
          type: 'doc',
          id: 'module-1/index',
        },
      },
      {
        type: 'category',
        label: 'Module 2 - The Digital Twin (Gazebo & Unity)',
        items: [
          'module-2/chapter-2-1',
          'module-2/chapter-2-2',
          'module-2/chapter-2-3',
        ],
        collapsed: true,
        collapsible: true,
        link: {
          type: 'doc',
          id: 'module-2/index',
        },
      },
      {
        type: 'category',
        label: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac™)',
        items: [
          'module-3/chapter-3-1',
          'module-3/chapter-3-2',
          'module-3/chapter-3-3',
        ],
        collapsed: true,
        collapsible: true,
        link: {
          type: 'doc',
          id: 'module-3/index',
        },
      },
      {
        type: 'category',
        label: 'Module 4 - Vision-Language-Action (VLA)',
        items: [
          'module-04/voice-to-action',
          'module-04/cognitive-planning',
          'module-04/autonomous-capstone',
        ],
        collapsed: true,
        collapsible: true,
        link: {
          type: 'doc',
          id: 'module-04/index',
        },
      },
    ],
  };

  export default sidebars;