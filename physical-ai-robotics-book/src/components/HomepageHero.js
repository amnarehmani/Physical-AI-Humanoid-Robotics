import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Heading from '@theme/Heading';
import styles from './HomepageHero.module.css';

export default function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.nebulaBackground}></div>
      <div className={clsx("container", styles.heroContainer)}>
        <div className={styles.heroText}>
          <Heading as="h1" className={styles.heroTitle}>
            PHYSICAL <span className={styles.highlight}>AI</span> & <br/>
            HUMANOID <span className={styles.highlight}>ROBOTICS</span>
          </Heading>
          <p className={styles.heroSubtitle}>
            Bridging the Gap Between <span className={styles.textGlow}>Simulation</span> and <span className={styles.textGlow}>Reality</span>
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/preface">
              ENTER THE FUTURE
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}