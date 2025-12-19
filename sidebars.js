// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/index',
        'module-1-ros2/middleware',
        'module-1-ros2/nodes-topics-services-actions',
        'module-1-ros2/python-agents',
        'module-1-ros2/urdf-overview',
        'module-1-ros2/learning-outcomes'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-simulation/index',
        'module-2-simulation/principles',
        'module-2-simulation/gazebo-unity',
        'module-2-simulation/components',
        'module-2-simulation/learning-outcomes'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-isaac/index',
        'module-3-isaac/isaac-sim',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/locomotion',
        'module-3-isaac/learning-outcomes'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/index',
        'module-4-vla/vla-concepts',
        'module-4-vla/voice-to-action',
        'module-4-vla/llm-planning',
        'module-4-vla/learning-outcomes'
      ],
    },
    {
      type: 'category',
      label: 'Capstone: The Autonomous Humanoid',
      items: [
        'capstone/index',
        'capstone/voice-perception',
        'capstone/perception-planning',
        'capstone/planning-navigation',
        'capstone/navigation-manipulation',
        'capstone/implementation-guide',
        'capstone/validation'
      ],
    },
    {
      type: 'category',
      label: 'Hardware Requirements & Lab Architecture',
      items: [
        'hardware-lab/index',
        'hardware-lab/lab-architecture',
        'hardware-lab/gpu-rigs',
        'hardware-lab/jetson-kits',
        'hardware-lab/sensors',
        'hardware-lab/semester-template',
        'hardware-lab/course-adaptation',
        'hardware-lab/exercises'
      ],
    },
    {
      type: 'category',
      label: 'Conclusion: The Future of Physical AI',
      items: [
        'conclusion/index',
        'conclusion/future',
        'conclusion/next-steps',
        'conclusion/resources',
        'conclusion/glossary'
      ],
    },
    {
      type: 'category',
      label: 'Testing',
      items: [
        'personalization-test',
        'translation-test'
      ],
    },
  ],
};

export default sidebars;