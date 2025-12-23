// src/pages/index.tsx

import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '../components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader(): React.ReactElement {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            <strong>
              START LEARNING <br />37 CHAPTERS
            </strong>
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): React.ReactElement {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="An interactive AI-powered book platform featuring 37 chapters across 4 modules covering ROS 2, Gazebo/Unity simulation, NVIDIA Isaac, and Vision-Language-Action models">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
      <section className={styles.authorSection}>
        <div className="container">
          <h2>About the Author</h2>
          <div className={styles.authorInfo}>
            <p><strong>Author:</strong> Azmat Ali</p>
            <p><strong>Contact:</strong> <a href="mailto:azmataliakbar@gmail.com">azmataliakbar@gmail.com</a></p>
            <p>This comprehensive guide covers the fundamentals and advanced concepts of robotics, from ROS 2 basics to cutting-edge Vision-Language-Action models.</p>
          </div>
        </div>
      </section>
    </Layout>
  );
}