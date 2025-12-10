import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './HomepageDynamicCTA.module.css';

export default function HomepageDynamicCTA() {
  return (
    <section className={clsx('hero hero--info', styles.dynamicCTA)}>
      <div className="container text--center">
        <Heading as="h2" className={styles.ctaHeading}>
          Ready to Build the Future of Robotics?
        </Heading>
        <p className={styles.ctaText}>
          Dive deep into the world of Physical AI and Humanoid Robotics. Start your journey today!
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--primary button--lg"
            to="/#modules-section">
            Start Learning Now
          </Link>
        </div>
      </div>
    </section>
  );
}