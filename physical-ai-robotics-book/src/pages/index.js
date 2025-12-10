import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import HomepageHero from '../components/HomepageHero';
import HomepageModules from '../components/HomepageModules';
import HomepageDynamicCTA from '../components/HomepageDynamicCTA';
import HomepageAbout from '../components/HomepageAbout';

const modules = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Learn ROS 2 architecture, Python control (rclpy), and URDF for robot structures, forming the robot\'s basic communication and physical definition.',
    link: '/docs/module-1/00-intro',
    icon: 'img/icon-ros2.svg',
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: 'Build robust virtual replicas using Gazebo for realistic physics and Unity for high-fidelity visualization and human-robot interaction.',
    link: '/docs/module-2/00-intro',
    icon: 'img/icon-sim.svg',
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    description: 'Upgrade to advanced AI capabilities with NVIDIA Isaac Sim for photorealistic simulation, Isaac ROS for accelerated perception, and Nav2 for autonomous navigation.',
    link: '/docs/module-3/00-intro',
    icon: 'img/icon-brain.svg',
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Integrate Large Language Models (LLMs) with robotics, enabling voice commands, cognitive planning, and complex autonomous actions.',
    link: '/docs/module-4/intro', // ✅ corrected path
    icon: 'img/icon-vla.svg',
  },
];

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description={siteConfig.tagline}>
      <HomepageHero />
      <main>
        <HomepageModules modules={modules} />
        <HomepageDynamicCTA />
        <HomepageAbout />
      </main>
    </Layout>
  );
}
