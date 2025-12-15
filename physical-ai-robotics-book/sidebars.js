// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'preface',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: The Basics',
          items: [
            'module-1/00-intro',
            'module-1/01-lesson-1',
            'module-1/02-lesson-2',
            'module-1/03-lesson-3',
            'module-1/04-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Advanced ROS 2',
          items: [
            'module-1/chapter-02-advanced-ros/m1-ch2-intro',
            'module-1/chapter-02-advanced-ros/m1-ch2-services',
            'module-1/chapter-02-advanced-ros/m1-ch2-actions',
            'module-1/chapter-02-advanced-ros/m1-ch2-parameters',
            'module-1/chapter-02-advanced-ros/m1-ch2-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Interfaces & Transforms',
          items: [
            'module-1/chapter-03-custom-interfaces/m1-ch3-intro',
            'module-1/chapter-03-custom-interfaces/m1-ch3-msgs',
            'module-1/chapter-03-custom-interfaces/m1-ch3-tf2',
            'module-1/chapter-03-custom-interfaces/m1-ch3-rviz',
            'module-1/chapter-03-custom-interfaces/m1-ch3-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: The ROS 2 Ecosystem',
          items: [
            'module-1/chapter-04-ecosystem/m1-ch4-intro',
            'module-1/chapter-04-ecosystem/m1-ch4-bag',
            'module-1/chapter-04-ecosystem/m1-ch4-debugging',
            'module-1/chapter-04-ecosystem/m1-ch4-security',
            'module-1/chapter-04-ecosystem/m1-ch4-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Project - TurtleBot3 Patrol',
          items: [
            'module-1/chapter-05-project/m1-ch5-intro',
            'module-1/chapter-05-project/m1-ch5-nav2',
            'module-1/chapter-05-project/m1-ch5-fsm',
            'module-1/chapter-05-project/m1-ch5-launch',
            'module-1/chapter-05-project/m1-ch5-summary',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: The Basics',
          items: [
            'module-2/00-intro',
            'module-2/01-lesson-1',
            'module-2/02-lesson-2',
            'module-2/03-lesson-3',
            'module-2/04-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Physics & Dynamics',
          items: [
            'module-2/chapter-02-physics/m2-ch2-intro',
            'module-2/chapter-02-physics/m2-ch2-inertial',
            'module-2/chapter-02-physics/m2-ch2-friction',
            'module-2/chapter-02-physics/m2-ch2-dynamics',
            'module-2/chapter-02-physics/m2-ch2-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Advanced Sensors',
          items: [
            'module-2/chapter-03-sensors/m2-ch3-intro',
            'module-2/chapter-03-sensors/m2-ch3-camera',
            'module-2/chapter-03-sensors/m2-ch3-lidar',
            'module-2/chapter-03-sensors/m2-ch3-imu',
            'module-2/chapter-03-sensors/m2-ch3-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: The ROS 2 - Gazebo Bridge',
          items: [
            'module-2/chapter-04-bridge/m2-ch4-intro',
            'module-2/chapter-04-bridge/m2-ch4-bridge',
            'module-2/chapter-04-bridge/m2-ch4-spawning',
            'module-2/chapter-04-bridge/m2-ch4-clock',
            'module-2/chapter-04-bridge/m2-ch4-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Project - Warehouse Digital Twin',
          items: [
            'module-2/chapter-05-project/m2-ch5-intro',
            'module-2/chapter-05-project/m2-ch5-world',
            'module-2/chapter-05-project/m2-ch5-multi-robot',
            'module-2/chapter-05-project/m2-ch5-fault-injection',
            'module-2/chapter-05-project/m2-ch5-summary',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: The Basics',
          items: [
            'module-3/00-intro',
            'module-3/01-lesson-1',
            'module-3/02-lesson-2',
            'module-3/03-lesson-3',
            'module-3/04-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Universal Scene Description (USD)',
          items: [
            'module-3/chapter-02-usd/m3-ch2-intro',
            'module-3/chapter-02-usd/m3-ch2-prims',
            'module-3/chapter-02-usd/m3-ch2-references',
            'module-3/chapter-02-usd/m3-ch2-physics',
            'module-3/chapter-02-usd/m3-ch2-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Synthetic Data Generation (Replicator)',
          items: [
            'module-3/chapter-03-replicator/m3-ch3-intro',
            'module-3/chapter-03-replicator/m3-ch3-randomization',
            'module-3/chapter-03-replicator/m3-ch3-annotators',
            'module-3/chapter-03-replicator/m3-ch3-scatter',
            'module-3/chapter-03-replicator/m3-ch3-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Project - Isaac Sim Navigation',
          items: [
            'module-3/chapter-04-project/m3-ch4-intro',
            'module-3/chapter-04-project/m3-ch4-loading',
            'module-3/chapter-04-project/m3-ch4-control',
            'module-3/chapter-04-project/m3-ch4-lidar',
            'module-3/chapter-04-project/m3-ch4-summary',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: The Basics',
          items: [
            'module-4/intro',
            'module-4/lesson-1',
            'module-4/lesson-2',
            'module-4/lesson-3',
            'module-4/summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Vision Encoders & CLIP',
          items: [
            'module-4/chapter-02-vision/m4-ch2-intro',
            'module-4/chapter-02-vision/m4-ch2-vit',
            'module-4/chapter-02-vision/m4-ch2-clip',
            'module-4/chapter-02-vision/m4-ch2-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Language Agents & Grounding',
          items: [
            'module-4/chapter-03-language/m4-ch3-intro',
            'module-4/chapter-03-language/m4-ch3-prompting',
            'module-4/chapter-03-language/m4-ch3-react',
            'module-4/chapter-03-language/m4-ch3-summary',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Project - The VLA Agent',
          items: [
            'module-4/chapter-04-project/m4-ch4-intro',
            'module-4/chapter-04-project/m4-ch4-pipeline',
            'module-4/chapter-04-project/m4-ch4-execution',
            'module-4/chapter-04-project/m4-ch4-summary',
          ],
        },
      ],
    },
    'appendix',
  ],
};

export default sidebars;
