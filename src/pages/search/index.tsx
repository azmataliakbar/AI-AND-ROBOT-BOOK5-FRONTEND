// src/pages/search/index.tsx

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import styles from './search.module.css';

type SearchResult = {
  id: string;
  title: string;
  content_preview: string;
  module: number;
  link: string;
};

export default function Search(): React.ReactElement {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<SearchResult[]>([]);
  const [hasSearched, setHasSearched] = useState(false);

  useEffect(() => {
    const urlParams = new URLSearchParams(window.location.search);
    const q = urlParams.get('q');
    if (q) {
      setQuery(q);
      performSearch(q);
    }
  }, []);

  const performSearch = (searchQuery: string) => {
    setHasSearched(true);
    
    // COMPLETE CHAPTER DATABASE - All 37 Chapters
    const allChapters: SearchResult[] = [
      // MODULE 1: ROS 2 Fundamentals (Chapters 1-10)
      {
        id: 'ch1',
        title: 'Chapter 1: Introduction to ROS 2 & Physical AI',
        content_preview: 'Understand the foundations of Physical AI and why ROS 2 is the nervous system of modern robots. Learn about nodes, topics, and DDS middleware.',
        module: 1,
        link: '/docs/module1/chapter01'
      },
      {
        id: 'ch2',
        title: 'Chapter 2: ROS 2 Core Concepts',
        content_preview: 'Master the fundamentals of ROS 2 architecture, communication patterns, and quality of service QoS policies.',
        module: 1,
        link: '/docs/module1/chapter02'
      },
      {
        id: 'ch3',
        title: 'Chapter 3: Nodes & Communication Patterns',
        content_preview: 'Deep dive into ROS 2 nodes, publishers, subscribers, and distributed system communication.',
        module: 1,
        link: '/docs/module1/chapter03'
      },
      {
        id: 'ch4',
        title: 'Chapter 4: Topics, Services & Actions',
        content_preview: 'Learn topics for streaming data, services for request-response, and actions for long-running tasks.',
        module: 1,
        link: '/docs/module1/chapter04'
      },
      {
        id: 'ch5',
        title: 'Chapter 5: Custom Messages & Interfaces',
        content_preview: 'Define domain-specific messages and interfaces for humanoid robot control and sensor data.',
        module: 1,
        link: '/docs/module1/chapter05'
      },
      {
        id: 'ch6',
        title: 'Chapter 6: Launch Files & Parameters',
        content_preview: 'Automate robot startup with launch files and configure behavior with dynamic parameters.',
        module: 1,
        link: '/docs/module1/chapter06'
      },
      {
        id: 'ch7',
        title: 'Chapter 7: TF2 & Coordinate Transformations',
        content_preview: 'Master coordinate frame transformations for robot kinematics, sensor fusion, and spatial reasoning.',
        module: 1,
        link: '/docs/module1/chapter07'
      },
      {
        id: 'ch8',
        title: 'Chapter 8: URDF for Humanoid Robots',
        content_preview: 'Learn URDF Unified Robot Description Format to model humanoid robot kinematics, joints, and visual appearance.',
        module: 1,
        link: '/docs/module1/chapter08'
      },
      {
        id: 'ch9',
        title: 'Chapter 9: ROS 2 Package Development',
        content_preview: 'Build professional ROS 2 packages with proper structure, dependencies, and build systems CMake ament.',
        module: 1,
        link: '/docs/module1/chapter09'
      },
      {
        id: 'ch10',
        title: 'Chapter 10: Building Python Controllers',
        content_preview: 'Connect Python AI agents to ROS 2 for intelligent robot control using rclpy.',
        module: 1,
        link: '/docs/module1/chapter10'
      },
      
      // MODULE 2: Gazebo & Unity Simulation (Chapters 11-18)
      {
        id: 'ch11',
        title: 'Chapter 11: Introduction to Gazebo Simulation',
        content_preview: 'Get started with Gazebo for physics-based robot simulation. Learn about worlds, models, and plugins.',
        module: 2,
        link: '/docs/module2/chapter11'
      },
      {
        id: 'ch12',
        title: 'Chapter 12: Physics Simulation in Gazebo',
        content_preview: 'Understand gravity, collisions, friction, and contact dynamics in Gazebo physics engine.',
        module: 2,
        link: '/docs/module2/chapter12'
      },
      {
        id: 'ch13',
        title: 'Chapter 13: Sensor Simulation',
        content_preview: 'Simulate realistic sensor data including cameras, LiDAR, depth sensors, and IMUs for robot perception.',
        module: 2,
        link: '/docs/module2/chapter13'
      },
      {
        id: 'ch14',
        title: 'Chapter 14: URDF & SDF Robot Models',
        content_preview: 'Compare URDF and SDF formats for robot description. Convert between formats and optimize for simulation.',
        module: 2,
        link: '/docs/module2/chapter14'
      },
      {
        id: 'ch15',
        title: 'Chapter 15: Unity for Robotics',
        content_preview: 'Use Unity for high-fidelity rendering, VR AR integration, and human-robot interaction scenarios.',
        module: 2,
        link: '/docs/module2/chapter15'
      },
      {
        id: 'ch16',
        title: 'Chapter 16: High-Fidelity Rendering',
        content_preview: 'Create photorealistic robot simulations with advanced lighting, materials, and post-processing.',
        module: 2,
        link: '/docs/module2/chapter16'
      },
      {
        id: 'ch17',
        title: 'Chapter 17: VR/AR Human-Robot Interaction',
        content_preview: 'Build immersive VR AR interfaces for robot teleoperation and interaction design.',
        module: 2,
        link: '/docs/module2/chapter17'
      },
      {
        id: 'ch18',
        title: 'Chapter 18: Sim-to-Real Transfer',
        content_preview: 'Bridge the reality gap with domain randomization, system identification, and transfer learning techniques.',
        module: 2,
        link: '/docs/module2/chapter18'
      },
      
      // MODULE 3: NVIDIA Isaac (Chapters 19-28)
      {
        id: 'ch19',
        title: 'Chapter 19: NVIDIA Isaac Platform Overview',
        content_preview: 'Discover NVIDIA Isaac comprehensive robotics platform combining simulation, SDK, and hardware acceleration.',
        module: 3,
        link: '/docs/module3/chapter19'
      },
      {
        id: 'ch20',
        title: 'Chapter 20: Isaac Sim Setup & Configuration',
        content_preview: 'Install and configure Isaac Sim for photorealistic robot simulation with GPU acceleration.',
        module: 3,
        link: '/docs/module3/chapter20'
      },
      {
        id: 'ch21',
        title: 'Chapter 21: Photorealistic Environments',
        content_preview: 'Create stunning photorealistic environments with ray tracing, global illumination, and physically-based rendering.',
        module: 3,
        link: '/docs/module3/chapter21'
      },
      {
        id: 'ch22',
        title: 'Chapter 22: Synthetic Data Generation',
        content_preview: 'Generate massive labeled datasets for AI training using Isaac Replicator and domain randomization.',
        module: 3,
        link: '/docs/module3/chapter22'
      },
      {
        id: 'ch23',
        title: 'Chapter 23: Isaac ROS Perception',
        content_preview: 'Implement GPU-accelerated perception pipelines for real-time object detection, tracking, and segmentation.',
        module: 3,
        link: '/docs/module3/chapter23'
      },
      {
        id: 'ch24',
        title: 'Chapter 24: Hardware-Accelerated VSLAM',
        content_preview: 'Deploy visual SLAM on GPU for real-time localization and mapping in complex environments.',
        module: 3,
        link: '/docs/module3/chapter24'
      },
      {
        id: 'ch25',
        title: 'Chapter 25: Nav2 Path Planning',
        content_preview: 'Implement Nav2 navigation stack for bipedal humanoid path planning and obstacle avoidance.',
        module: 3,
        link: '/docs/module3/chapter25'
      },
      {
        id: 'ch26',
        title: 'Chapter 26: Bipedal Locomotion Control',
        content_preview: 'Master humanoid walking controllers, balance, and dynamic gait generation techniques.',
        module: 3,
        link: '/docs/module3/chapter26'
      },
      {
        id: 'ch27',
        title: 'Chapter 27: Manipulation & Grasping',
        content_preview: 'Implement robot manipulation, grasp planning, and dexterous manipulation for humanoid hands.',
        module: 3,
        link: '/docs/module3/chapter27'
      },
      {
        id: 'ch28',
        title: 'Chapter 28: Deployment to Jetson',
        content_preview: 'Deploy AI models and perception pipelines to NVIDIA Jetson edge devices for real-time inference.',
        module: 3,
        link: '/docs/module3/chapter28'
      },
      
      // MODULE 4: Vision-Language-Action Models (Chapters 29-37)
      {
        id: 'ch29',
        title: 'Chapter 29: Vision-Language-Action Models',
        content_preview: 'Introduction to VLA models RT-1, RT-2, OpenVLA that combine vision, language, and actions for embodied AI.',
        module: 4,
        link: '/docs/module4/chapter29'
      },
      {
        id: 'ch30',
        title: 'Chapter 30: Multimodal Transformers',
        content_preview: 'Understand transformer architectures for processing vision, language, and action data simultaneously.',
        module: 4,
        link: '/docs/module4/chapter30'
      },
      {
        id: 'ch31',
        title: 'Chapter 31: OpenAI Whisper Integration',
        content_preview: 'Integrate Whisper speech recognition for voice commands and natural robot interaction.',
        module: 4,
        link: '/docs/module4/chapter31'
      },
      {
        id: 'ch32',
        title: 'Chapter 32: GPT-4 for Robot Control',
        content_preview: 'Use GPT-4 for high-level task planning, translating natural language to robot actions.',
        module: 4,
        link: '/docs/module4/chapter32'
      },
      {
        id: 'ch33',
        title: 'Chapter 33: Cognitive Task Planning',
        content_preview: 'Implement cognitive planning systems that break down complex tasks into executable action sequences.',
        module: 4,
        link: '/docs/module4/chapter33'
      },
      {
        id: 'ch34',
        title: 'Chapter 34: Natural Language to Actions',
        content_preview: 'Map language commands like clean the room to precise ROS 2 action sequences.',
        module: 4,
        link: '/docs/module4/chapter34'
      },
      {
        id: 'ch35',
        title: 'Chapter 35: Human-Robot Interaction',
        content_preview: 'Design natural interaction paradigms using speech, gesture, and multimodal communication.',
        module: 4,
        link: '/docs/module4/chapter35'
      },
      {
        id: 'ch36',
        title: 'Chapter 36: Voice-Controlled Humanoids',
        content_preview: 'Build complete voice-controlled humanoid systems integrating Whisper, GPT-4, and ROS 2.',
        module: 4,
        link: '/docs/module4/chapter36'
      },
      {
        id: 'ch37',
        title: 'Chapter 37: Capstone - Autonomous Humanoid',
        content_preview: 'Final capstone project: Build a fully autonomous humanoid with voice control, navigation, manipulation, and AI planning.',
        module: 4,
        link: '/docs/module4/chapter37'
      },
    ];

    // Search through title AND content
    const filtered = allChapters.filter(result => 
      result.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
      result.content_preview.toLowerCase().includes(searchQuery.toLowerCase())
    );

    setResults(filtered);
  };

  const handleSearch = () => {
    performSearch(query);
  };

  const getModuleName = (moduleNum: number): string => {
    const names: Record<number, string> = {
      1: 'Module 1: ROS 2',
      2: 'Module 2: Simulation',
      3: 'Module 3: NVIDIA Isaac',
      4: 'Module 4: VLA Models'
    };
    return names[moduleNum] || 'Unknown';
  };

  return (
    <Layout title="Search">
      <div className={styles.searchContainer}>
        <div className={styles.searchHeader}>
          <h1>🔍 Search the Book</h1>
          {query && <p>Searching for: <strong>{query}</strong></p>}
        </div>

        <div className={styles.searchBox}>
          <input
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            onKeyPress={(e) => e.key === 'Enter' && handleSearch()}
            placeholder="Search chapters, ROS 2, Gazebo, URDF, Isaac, VLA..."
            className={styles.searchInput}
            autoFocus
          />
          <button onClick={handleSearch} className={styles.searchButton}>
            🔍 Search
          </button>
        </div>

        {hasSearched && results.length > 0 && (
          <div className={styles.results}>
            <h2>Found {results.length} result{results.length !== 1 ? 's' : ''}</h2>
            {results.map((result) => (
              <a key={result.id} href={result.link} className={styles.resultCard}>
                <div className={styles.resultHeader}>
                  <span className={styles.moduleTag} data-module={result.module}>
                    {getModuleName(result.module)}
                  </span>
                </div>
                <h3 className={styles.resultTitle}>{result.title}</h3>
                <p className={styles.resultPreview}>{result.content_preview}</p>
                <div className={styles.resultFooter}>
                  <span className={styles.readMore}>Read Chapter →</span>
                </div>
              </a>
            ))}
          </div>
        )}

        {hasSearched && results.length === 0 && (
          <div className={styles.noResults}>
            <div className={styles.noResultsIcon}>🔍</div>
            <h3>No results found</h3>
            <p>Try different keywords like "ROS 2", "Gazebo", "URDF", "Isaac", or "VLA"</p>
          </div>
        )}

        {!hasSearched && (
          <div className={styles.popularTopics}>
            <h2>🔥 Popular Topics</h2>
            <div className={styles.topicGrid}>
              {['ROS 2', 'Gazebo', 'URDF', 'Isaac', 'VLA', 'Python', 'Unity', 'Whisper', 'GPT-4', 'Navigation'].map(topic => (
                <button
                  key={topic}
                  onClick={() => {
                    setQuery(topic);
                    setTimeout(() => performSearch(topic), 100);
                  }}
                  className={styles.topicButton}
                >
                  {topic}
                </button>
              ))}
            </div>
          </div>
        )}
      </div>
    </Layout>
  );
}