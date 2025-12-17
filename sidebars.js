// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'ros2-module/intro',
    {
      type: 'category',
      label: 'ROS2 Robotic Nervous System',
      items: [
        {
          type: 'category',
          label: 'Core Concepts',
          items: [
            'ros2-module/core-concepts/index',
            'ros2-module/core-concepts/purpose',
            'ros2-module/core-concepts/nodes-topics-services',
            'ros2-module/core-concepts/communication-flow'
          ],
        },
        {
          type: 'category',
          label: 'Python Agents with rclpy',
          items: [
            'ros2-module/python-agents/index',
            'ros2-module/python-agents/rclpy-basics',
            'ros2-module/python-agents/python-nodes',
            'ros2-module/python-agents/ai-robot-bridge',
            'ros2-module/python-agents/agent-actuator-communication'
          ],
        },
        {
          type: 'category',
          label: 'Humanoid Description with URDF',
          items: [
            'ros2-module/urdf-description/index',
            'ros2-module/urdf-description/urdf-structure',
            'ros2-module/urdf-description/links-joints-kinematics',
            'ros2-module/urdf-description/simulation-control'
          ],
        },
        {
          type: 'category',
          label: 'Appendices',
          items: [
            'ros2-module/glossary'
          ],
        }
      ],
    },
    {
      type: 'category',
      label: 'Digital Twin (Gazebo & Unity)',
      items: [
        'digital-twin/index',
        {
          type: 'category',
          label: 'Introduction to Digital Twins',
          items: [
            'digital-twin/intro',
            'digital-twin/introduction',
            'digital-twin/role-of-digital-twins',
            'digital-twin/benefits',
            'digital-twin/use-cases',
            'digital-twin/assessment-questions'
          ],
        },
        {
          type: 'category',
          label: 'Physics Simulation with Gazebo',
          items: [
            'digital-twin/physics-simulation/index',
            'digital-twin/physics-simulation/role-of-digital-twins',
            'digital-twin/physics-simulation/gravity-collisions-physics-engines',
            'digital-twin/physics-simulation/simulating-humanoid-motion',
            'digital-twin/physics-simulation/physics-simulation-fundamentals',
            'digital-twin/physics-simulation/gravity-modeling',
            'digital-twin/physics-simulation/collision-detection',
            'digital-twin/physics-simulation/physics-engines',
            'digital-twin/physics-simulation/humanoid-motion-simulation',
            'digital-twin/physics-simulation/gazebo-introduction',
            'digital-twin/physics-simulation/physics-assessment'
          ],
        },
        {
          type: 'category',
          label: 'Sensor Simulation',
          items: [
            'digital-twin/sensor-simulation/index',
            'digital-twin/sensor-simulation/purpose-of-sensor-simulation',
            'digital-twin/sensor-simulation/lidar-simulation',
            'digital-twin/sensor-simulation/depth-camera-simulation',
            'digital-twin/sensor-simulation/imu-simulation',
            'digital-twin/sensor-simulation/sensor-data-flow',
            'digital-twin/sensor-simulation/sensor-comparison',
            'digital-twin/sensor-simulation/sensor-assessment'
          ],
        },
        {
          type: 'category',
          label: 'Environment & Interaction in Unity',
          items: [
            'digital-twin/unity-interaction/index',
            'digital-twin/unity-interaction/high-fidelity-rendering',
            'digital-twin/unity-interaction/hri-concepts',
            'digital-twin/unity-interaction/visual-realism-vs-physical-accuracy',
            'digital-twin/unity-interaction/unity-examples',
            'digital-twin/unity-interaction/unity-setup',
            'digital-twin/unity-interaction/unity-assessment'
          ],
        },
        {
          type: 'category',
          label: 'Module Summary',
          items: [
            'digital-twin/glossary',
            'digital-twin/conclusion'
          ],
        }
      ],
    },
    {
      type: 'category',
      label: 'AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'nvidia-isaac-ai-robot/index',
        {
          type: 'category',
          label: 'NVIDIA Isaac Sim',
          items: [
            'nvidia-isaac-ai-robot/isaac-sim/overview',
            'nvidia-isaac-ai-robot/isaac-sim/photorealistic-simulation',
            'nvidia-isaac-ai-robot/isaac-sim/synthetic-data-generation',
            'nvidia-isaac-ai-robot/isaac-sim/training-vs-testing',
            'nvidia-isaac-ai-robot/isaac-sim/assessment'
          ],
        },
        {
          type: 'category',
          label: 'Isaac ROS Perception',
          items: [
            'nvidia-isaac-ai-robot/isaac-ros/overview',
            'nvidia-isaac-ai-robot/isaac-ros/vslam',
            'nvidia-isaac-ai-robot/isaac-ros/sensor-pipelines',
            'nvidia-isaac-ai-robot/isaac-ros/perception-stacks',
            'nvidia-isaac-ai-robot/isaac-ros/ros2-integration',
            'nvidia-isaac-ai-robot/isaac-ros/examples',
            'nvidia-isaac-ai-robot/isaac-ros/assessment'
          ],
        },
        {
          type: 'category',
          label: 'Nav2 Navigation',
          items: [
            'nvidia-isaac-ai-robot/nav2-humanoid/overview',
            'nvidia-isaac-ai-robot/nav2-humanoid/path-planning',
            'nvidia-isaac-ai-robot/nav2-humanoid/humanoid-movement',
            'nvidia-isaac-ai-robot/nav2-humanoid/navigation-pipelines',
            'nvidia-isaac-ai-robot/nav2-humanoid/bipedal-navigation',
            'nvidia-isaac-ai-robot/nav2-humanoid/examples',
            'nvidia-isaac-ai-robot/nav2-humanoid/assessment'
          ],
        },
        {
          type: 'category',
          label: 'Module Summary',
          items: [
            'nvidia-isaac-ai-robot/glossary',
            'nvidia-isaac-ai-robot/conclusion'
          ],
        }
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action',
      items: [
        'vla-module/index',
        {
          'VLA Paradigm': [
            'vla-module/vla-paradigm/overview',
            // additional sub-sections
          ],
        },
        {
          'Voice-to-Action': [
            'vla-module/voice-to-action/interfaces',
            // additional sub-sections
          ],
        },
        {
          'LLM Planning': [
            'vla-module/llm-planning/cognitive-planning',
            // additional sub-sections
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;