// HomePageFeatures.tsx

import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

type FeatureItem = {
  title: string;
  description: React.ReactElement;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Interactive Learning',
    description: (
      <>
        Engage with the content through interactive examples and hands-on exercises.
      </>
    ),
  },
  {
    title: 'AI-Powered Assistance',
    description: (
      <>
        Get instant answers to your questions with our AI-powered chat assistant.
      </>
    ),
  },
  {
    title: 'Progress Tracking',
    description: (
      <>
        Track your learning progress and pick up where you left off.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem): React.ReactElement {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): React.ReactElement {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}