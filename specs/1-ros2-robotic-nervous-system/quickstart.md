# Quick Start Guide: ROS2 Robotic Nervous System Module

**Feature**: 1-ros2-robotic-nervous-system
**Date**: 2025-12-17
**Status**: Complete

## Overview

This guide provides a quick path to get the ROS2 educational module up and running using Docusaurus. It covers the essential setup steps needed to build, develop, and deploy the educational content.

## Prerequisites

Before starting, ensure you have:

- **Node.js**: Version 18.0 or higher
- **npm** or **yarn**: Package manager for Node.js
- **Git**: Version control system
- **GitHub account**: For deployment to GitHub Pages (if applicable)

## Setup Instructions

### 1. Clone and Initialize the Repository

```bash
# Clone the repository (if not already done)
git clone <repository-url>
cd <repository-name>

# Install dependencies
npm install
# OR if using yarn
yarn install
```

### 2. Initialize Docusaurus

If this is a new Docusaurus project, initialize it:

```bash
# Install Docusaurus globally (if not already installed)
npm install -g @docusaurus/init

# Create a new Docusaurus site (if starting fresh)
npx @docusaurus/init@latest init docs classic

# Configure the project to use the docs directory
```

### 3. Add the ROS2 Module Content

```bash
# Create the module directory structure
mkdir -p docs/ros2-module/core-concepts
mkdir -p docs/ros2-module/python-agents
mkdir -p docs/ros2-module/urdf-description

# The content files will be created based on the specification
```

### 4. Run the Development Server

```bash
# Start the development server
npm run start
# OR if using yarn
yarn start

# The site will be available at http://localhost:3000
```

### 5. Build for Production

```bash
# Build the static site
npm run build
# OR if using yarn
yarn build

# The built site will be in the build/ directory
```

### 6. Deploy to GitHub Pages

```bash
# Deploy to GitHub Pages
npm run deploy
# OR if using yarn
yarn deploy

# This will push the built site to the gh-pages branch
```

## Content Structure

The ROS2 module follows this structure:

```
docs/
└── ros2-module/
    ├── intro.md                 # Module introduction
    ├── core-concepts/           # Chapter 1: ROS 2 Core Concepts
    │   ├── index.md            # Chapter overview
    │   ├── purpose.md          # ROS 2 purpose
    │   ├── nodes-topics-services.md  # Core communication patterns
    │   └── communication-flow.md     # Robot communication flow
    ├── python-agents/          # Chapter 2: Python Agents with rclpy
    │   ├── index.md            # Chapter overview
    │   ├── rclpy-basics.md     # Python-based ROS 2 nodes
    │   ├── ai-robot-bridge.md  # Bridging AI logic to robot controllers
    │   └── agent-actuator-communication.md  # Agent-to-actuator communication
    └── urdf-description/       # Chapter 3: Humanoid Description with URDF
        ├── index.md            # Chapter overview
        ├── urdf-structure.md   # URDF role and structure
        ├── links-joints-kinematics.md  # Links, joints, kinematics
        └── simulation-control.md       # Connection to simulation and control
```

## Key Configuration Files

### docusaurus.config.js
- Main configuration file for the Docusaurus site
- Contains site metadata, theme configuration, and plugin settings
- Defines the sidebar navigation structure

### sidebar.js
- Defines the navigation structure for the documentation
- Organizes content in a logical hierarchy
- Makes it easy for students to navigate the module

## Development Workflow

1. **Edit content**: Modify Markdown files in the docs/ directory
2. **Preview changes**: Use `npm run start` to see changes in real-time
3. **Validate content**: Ensure all content aligns with the specification
4. **Test navigation**: Verify all links and cross-references work correctly
5. **Build and deploy**: When ready, build and deploy to production

## Common Commands

```bash
# Start development server
npm run start

# Build for production
npm run build

# Deploy to GitHub Pages
npm run deploy

# Serve built site locally (for testing)
npm run serve

# Clear build cache
npm run clear
```

## Troubleshooting

### Common Issues

**Issue**: Site doesn't reflect changes after editing Markdown files
- **Solution**: Restart the development server with `npm run start`

**Issue**: Links between pages don't work correctly
- **Solution**: Verify relative paths are correct and use Docusaurus link syntax: `[text](/path/to/page)`

**Issue**: Images or other assets don't load
- **Solution**: Place assets in the `static/` directory and reference with absolute paths

### Getting Help

- Check the [Docusaurus documentation](https://docusaurus.io/docs)
- Review the [original specification](spec.md) for content requirements
- Consult the [implementation plan](plan.md) for architectural decisions