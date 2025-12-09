import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './HomepageAbout.module.css';

export default function HomepageAbout() {
  return (
    <section id="about-book" className={styles.aboutSection}>
      <div className={styles.parallaxBg}></div>
      <div className={clsx("container", styles.container)}>
        <div className={styles.contentWrapper}>
          <div className={styles.textContent}>
            <Heading as="h2" className={styles.sectionTitle}>
              ENGINEERING <span className={styles.highlight}>INTELLIGENCE</span>
            </Heading>
            <div className={styles.divider}></div>
            <p className={styles.text}>
              "Physical AI & Humanoid Robotics" explores the exciting intersection of advanced AI and embodied robotics. 
              From fundamental ROS 2 concepts to cutting-edge NVIDIA Isaac simulations and Vision-Language-Action models, 
              this book provides a comprehensive guide for aspiring roboticists and AI engineers.
            </p>
            <p className={styles.text}>
              It emphasizes a <span className={styles.highlightText}>"Simulation-First"</span> approach, empowering learners to develop, test, and validate 
              complex robotic behaviors in virtual environments before deploying to expensive hardware. 
              Discover how to build intelligent, autonomous systems that can perceive, reason, and act in the physical world.
            </p>

            <div className={styles.statsRow}>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>04</span>
                <span className={styles.statLabel}>Core Modules</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>20+</span>
                <span className={styles.statLabel}>Projects</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statNumber}>∞</span>
                <span className={styles.statLabel}>Possibilities</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}
