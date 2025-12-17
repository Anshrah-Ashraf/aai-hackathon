# AI-Spec-Driven Book with Embedded RAG Chatbot - Frontend

This repository contains the frontend implementation of the AI-Spec-Driven Book with Embedded RAG Chatbot, specifically focusing on the ROS2 Robotic Nervous System module.

## Overview

This Docusaurus-based educational module introduces ROS 2 as the middleware connecting AI agents to humanoid robots. It's designed for CS/AI students with basic Python knowledge and covers:

1. **ROS 2 Core Concepts**: Understanding the purpose, nodes, topics, and services
2. **Python Agents with rclpy**: Creating Python-based ROS 2 nodes that bridge AI logic to robot controllers
3. **Humanoid Description with URDF**: Understanding URDF role, structure, links, joints, and kinematics

## Features

- Comprehensive educational content on ROS2 concepts
- Practical examples and code snippets
- Integration with AI-agent communication patterns
- Humanoid robot modeling with URDF
- Simulation and control system integration

## Installation

1. Make sure you have Node.js installed (version 18.0 or higher)
2. Clone the repository
3. Navigate to the `front_end_book` directory
4. Install dependencies:

```bash
npm install
```

## Running the Development Server

To start the development server:

```bash
cd front_end_book
npm start
```

This will start the Docusaurus development server on `http://localhost:3000`.

## Building for Production

To build the static site for deployment:

```bash
cd front_end_book
npm run build
```

## Project Structure

```
front_end_book/
├── docs/
│   └── ros2-module/          # ROS2 educational content
│       ├── intro.md          # Introduction to the module
│       ├── core-concepts/    # ROS2 core concepts chapter
│       ├── python-agents/    # Python agents with rclpy chapter
│       └── urdf-description/ # URDF description chapter
├── src/
│   ├── css/                  # Custom styles
│   ├── pages/                # Custom pages
│   └── components/           # React components
├── docusaurus.config.js      # Docusaurus configuration
├── sidebars.js               # Navigation sidebar configuration
├── package.json              # Project dependencies and scripts
└── README.md                 # This file
```

## Documentation Sections

- **Introduction**: Overview of the ROS2 Robotic Nervous System
- **Core Concepts**: Fundamental ROS2 concepts including nodes, topics, and services
- **Python Agents**: Using rclpy to create Python-based ROS2 nodes
- **URDF Description**: Understanding robot description with URDF
- **Glossary**: Key terms and definitions

## Technologies Used

- [Docusaurus](https://docusaurus.io/): Static site generator for documentation
- [React](https://reactjs.org/): Component-based UI library
- [Node.js](https://nodejs.org/): JavaScript runtime environment
- [npm](https://www.npmjs.com/): Package manager

## Contributing

To contribute to this educational module:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

If you encounter any issues or have questions about the content, please open an issue in the repository.