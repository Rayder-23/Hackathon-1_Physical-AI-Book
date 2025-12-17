import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning - 10min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function ModuleOverview() {
  const modules = [
    {
      id: 1,
      title: 'The Robotic Nervous System (ROS 2)',
      description: 'Learn about the middleware architecture that connects all robotic components together',
      path: '/docs/module-1-ros2',
      icon: '🤖'
    },
    {
      id: 2,
      title: 'The Digital Twin (Gazebo & Unity)',
      description: 'Explore simulation environments that mirror the physical world for testing and development',
      path: '/docs/module-2-simulation',
      icon: '🌐'
    },
    {
      id: 3,
      title: 'The AI-Robot Brain (NVIDIA Isaac)',
      description: 'Discover AI-powered perception and navigation systems for autonomous robots',
      path: '/docs/module-3-isaac',
      icon: '🧠'
    },
    {
      id: 4,
      title: 'Vision-Language-Action (VLA)',
      description: 'Understand how robots integrate perception, cognition, and action in embodied AI systems',
      path: '/docs/module-4-vla',
      icon: '👁️'
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container padding-horiz--md">
        <Heading as="h2" className="text--center margin-bottom--xl">
          Four Core Modules
        </Heading>
        <div className="row">
          {modules.map((module) => (
            <div key={module.id} className="col col--3 margin-bottom--lg">
              <div className="text--center padding--md" style={{height: '100%', border: '1px solid #ccc', borderRadius: '8px'}}>
                <div className="margin-bottom--sm" style={{fontSize: '2rem'}}>{module.icon}</div>
                <Heading as="h3">{module.title}</Heading>
                <p>{module.description}</p>
                <Link className="button button--primary button--sm" to={module.path}>
                  Explore Module
                </Link>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive guide to Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--4 col--offset-1">
                <div className="padding-horiz--md">
                  <Heading as="h2">About This Book</Heading>
                  <p>
                    This comprehensive guide explores Physical AI and Embodied Intelligence,
                    covering the complete pipeline from voice command to robotic action.
                    You'll learn how to build autonomous humanoid robots using modern AI techniques.
                  </p>
                  <p>
                    The book is structured around four core modules that form the foundation
                    of embodied AI systems. Each module builds upon the previous one to
                    provide a complete understanding of humanoid robotics.
                  </p>
                </div>
              </div>
              <div className="col col--5 col--offset-1">
                <div className="padding-horiz--md">
                  <Heading as="h2">Learning Outcomes</Heading>
                  <ul>
                    <li>Understand the fundamentals of Physical AI and Embodied Intelligence</li>
                    <li>Master ROS 2 for robotic control and communication</li>
                    <li>Work with simulation environments for robot development</li>
                    <li>Implement AI perception and navigation systems</li>
                    <li>Build Vision-Language-Action systems for autonomous behavior</li>
                    <li>Create complete autonomous humanoid pipelines</li>
                  </ul>
                  <div className="margin-top--lg">
                    <Link className="button button--primary button--lg" to="/docs/intro">
                      Begin Your Journey
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
        <ModuleOverview />
      </main>
    </Layout>
  );
}
