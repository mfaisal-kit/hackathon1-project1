import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './ModuleSlider.module.css';

const modules = [
  {
    id: 1,
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Learn about ROS 2 as the middleware connecting AI decision-making to physical humanoid robot control. Understand nodes, topics, services, and actions that enable communication between different parts of a robotic system.',
    image: 'img/undraw_docusaurus_mountain.svg',
    link: '/docs/modules/ros2/intro',
    color: '#a78bfa' // light purple
  },
  {
    id: 2,
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: 'Explore physics simulation with Gazebo and high-fidelity interaction with Unity. Learn how to create accurate digital representations of physical robots for testing and development.',
    image: 'img/undraw_docusaurus_tree.svg',
    link: '/docs/modules/gazebo-unity/intro',
    color: '#c4b5fd' // lighter purple
  },
  {
    id: 3,
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    description: 'Master NVIDIA Isaac Sim for synthetic data generation and Isaac ROS for perception and localization. Understand Nav2 for navigation systems in humanoid robotics.',
    image: 'img/undraw_docusaurus_react.svg',
    link: '/docs/modules/isaac/intro',
    color: '#ddd6fe' // very light purple
  },
  {
    id: 4,
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Integrate vision, language, and action systems for advanced humanoid robotics. Learn voice-to-action with speech models and language-driven cognitive planning.',
    image: 'img/undraw_docusaurus_react.svg',
    link: '/docs/modules/vla/intro',
    color: '#f5f3ff' // purple-tinged white
  }
];

const ModuleSlider = () => {
  const [currentIndex, setCurrentIndex] = useState(0);

  // Disable auto-rotation completely
  // No useEffect for auto-rotation

  const goToSlide = (index) => {
    setCurrentIndex(index);
  };

  const goToNext = () => {
    setCurrentIndex((prevIndex) => (prevIndex + 1) % modules.length);
  };

  const goToPrev = () => {
    setCurrentIndex((prevIndex) => (prevIndex - 1 + modules.length) % modules.length);
  };

  return (
    <div className={clsx(styles.sliderContainer, 'margin-vert--lg')}>
      <div className={styles.slider}>
        <div className={styles.slideTrack} style={{transform: `translateX(-${currentIndex * 100}%)`}}>
          {modules.map((module, index) => (
            <div 
              key={module.id} 
              className={clsx(styles.slide, {[styles.active]: index === currentIndex})}
              style={{backgroundColor: module.color}}
            >
              <div className="container">
                <div className="row">
                  <div className="col col--6">
                    <div className="padding-horiz--md">
                      <Heading as="h2">{module.title}</Heading>
                      <p className={styles.description}>{module.description}</p>
                      <Link className="button button--primary button--lg" to={module.link}>
                        Explore Module
                      </Link>
                    </div>
                  </div>
                  <div className="col col--6">
                    <div className="text--center padding-horiz--md">
                      <img 
                        src={require(`@site/static/${module.image}`).default} 
                        alt={module.title} 
                        className={styles.slideImage}
                        style={{filter: 'drop-shadow(0 10px 15px rgba(0, 0, 0, 0.1)'}}
                      />
                    </div>
                  </div>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
      
      {/* Slider controls */}
      <div className={styles.sliderControls}>
        <button 
          className={clsx(styles.navButton, styles.prevButton)} 
          onClick={goToPrev}
          aria-label="Previous module"
        >
          &#8249;
        </button>
        
        <div className={styles.indicators}>
          {modules.map((_, index) => (
            <button
              key={index}
              className={clsx(styles.indicator, {[styles.active]: index === currentIndex})}
              onClick={() => goToSlide(index)}
              aria-label={`Go to module ${index + 1}`}
            />
          ))}
        </div>
        
        <button 
          className={clsx(styles.navButton, styles.nextButton)} 
          onClick={goToNext}
          aria-label="Next module"
        >
          &#8250;
        </button>
      </div>
    </div>
  );
};

export default ModuleSlider;