// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'modules/ros2/intro',
        'modules/ros2/chapter-1/introduction-to-ros2-nervous-system',
        'modules/ros2/chapter-2/communicating-with-robots-using-ros2',
        'modules/ros2/chapter-3/humanoid-robot-structure-with-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'modules/gazebo-unity/intro',
        'modules/gazebo-unity/chapter-1/physics-simulation-with-gazebo',
        'modules/gazebo-unity/chapter-2/high-fidelity-interaction-with-unity',
        'modules/gazebo-unity/chapter-3/simulated-sensors-for-perception',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/isaac/intro',
        'modules/isaac/chapter-1/nvidia-isaac-sim-synthetic-data',
        'modules/isaac/chapter-2/isaac-ros-perception-localization',
        'modules/isaac/chapter-3/nav2-humanoid-navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'modules/vla/intro',
        'modules/vla/chapter-1/voice-to-action-with-speech-models',
        'modules/vla/chapter-2/language-driven-cognitive-planning',
        'modules/vla/chapter-3/capstone-autonomous-humanoid',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
