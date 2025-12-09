import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './HomepageModules.module.css';

function ModuleCard({title, description, link, icon, index}) {
  return (
    <Link to={link} className={clsx(styles.moduleCard)}>
      <div className={styles.cardContent}>
        <div className={styles.iconContainer}>
          {icon && <img src={icon} alt={title} className={styles.moduleIcon} />}
        </div>
        <div className={styles.textContainer}>
          <Heading as="h3" className={styles.moduleTitle}>
            {title}
          </Heading>
          <p className={styles.moduleDescription}>{description}</p>
          <span className={styles.learnMore}>
            Explore Module <span className={styles.arrow}>→</span>
          </span>
        </div>
        <div className={styles.numberOverlay}>0{index + 1}</div>
      </div>
      <div className={styles.glowBorder}></div>
    </Link>
  );
}

export default function HomepageModules({modules}) {
  return (
    <section className={styles.modulesSection} id="modules-overview">
      <div className="container">
        <div className={styles.headerWrapper}>
          <Heading as="h2" className={styles.sectionTitle}>
            SYSTEM <span className={styles.highlight}>MODULES</span>
          </Heading>
          <div className={styles.separator}></div>
        </div>
        <div className={styles.modulesGrid}>
          {modules.map((props, idx) => (
            <ModuleCard key={idx} {...props} index={idx} />
          ))}
        </div>
      </div>
    </section>
  );
}