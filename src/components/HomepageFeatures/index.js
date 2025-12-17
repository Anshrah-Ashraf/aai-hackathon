import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Comprehensive ROS2 Education',
    description: (
      <>
        Complete coverage of ROS2 core concepts, Python agents with rclpy,
        and humanoid robot description with URDF.
      </>
    ),
  },
  {
    title: 'AI-Robot Integration',
    description: (
      <>
        Learn how to connect AI agents to robotic systems using ROS2 as
        the middleware for seamless communication.
      </>
    ),
  },
  {
    title: 'Practical Examples',
    description: (
      <>
        Real-world examples and code snippets that demonstrate practical
        applications of ROS2 concepts.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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