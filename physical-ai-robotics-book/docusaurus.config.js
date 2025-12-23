// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging the Gap Between Simulation and Reality',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'facebook',
  projectName: 'physical-ai-robotics-book',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],


  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        { to: '/', label: 'Home', position: 'left' },
        { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Modules' },
        { to: '/#about-book', label: 'About', position: 'left' },
        { to: '/docs/chatbot-demo', label: 'Chatbot', position: 'left' },
        {
          href: 'https://github.com/your-org/physical-ai-robotics-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Navigation',
          items: [
            { label: 'Home', to: '/' },
            { label: 'About the Book', to: '/#about-book' },
            { label: 'Modules', to: '/docs/module-1/00-intro' },
            { label: 'Chatbot Demo', to: '/docs/chatbot-demo' },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Project.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },

  // Custom fields for your app, optional
  customFields: {
    BACKEND_URL: process.env.BACKEND_URL || 'http://127.0.0.1:8000',
  },
};

export default config;