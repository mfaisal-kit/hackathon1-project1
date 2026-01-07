import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn about ROS 2 as the middleware connecting AI decision-making to physical humanoid robot control.
        Understand nodes, topics, services, and actions that enable communication between different parts of a robotic system.
      </>
    ),
    link: '/docs/modules/ros2/intro'
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Explore physics simulation with Gazebo and high-fidelity interaction with Unity.
        Learn how to create accurate digital representations of physical robots for testing and development.
      </>
    ),
    link: '/docs/modules/gazebo-unity/intro'
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Master NVIDIA Isaac Sim for synthetic data generation and Isaac ROS for perception and localization.
        Understand Nav2 for navigation systems in humanoid robotics.
      </>
    ),
    link: '/docs/modules/isaac/intro'
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Integrate vision, language, and action systems for advanced humanoid robotics.
        Learn voice-to-action with speech models and language-driven cognitive planning.
      </>
    ),
    link: '/docs/modules/vla/intro'
  },
];

function Feature({Svg, title, description, link}) {
  return (
    <div className={clsx('col col--3')}>
      <div className={styles.featureCard}>
        <div className="text--center">
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
          <Link className="button button--secondary" to={link}>
            Explore Module
          </Link>
        </div>
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
