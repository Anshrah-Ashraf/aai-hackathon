# Quick Start Guide: Digital Twin Gazebo Unity Module

**Feature**: 2-digital-twin-gazebo-unity
**Date**: 2025-12-17
**Status**: Complete

## Overview

This guide provides a quick path to get the Digital Twin educational module up and running using Docusaurus. It covers the essential setup steps needed to build, develop, and deploy the educational content about digital twin environments for humanoid robots using physics simulation and high-fidelity rendering.

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

### 3. Add the Digital Twin Module Content

```bash
# Create the module directory structure
mkdir -p docs/digital-twin/physics-simulation
mkdir -p docs/digital-twin/unity-interaction
mkdir -p docs/digital-twin/sensor-simulation

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

The Digital Twin module follows this structure:

```
docs/
└── digital-twin/
    ├── intro.md                 # Module introduction
    ├── physics-simulation/      # Chapter 1: Physics Simulation with Gazebo
    │   ├── index.md            # Chapter overview
    │   ├── role-of-digital-twins.md  # Role of digital twins
    │   ├── gravity-collisions-physics-engines.md  # Gravity, collisions, physics engines
    │   └── simulating-humanoid-motion.md  # Simulating humanoid motion
    ├── unity-interaction/       # Chapter 2: Environment & Interaction in Unity
    │   ├── index.md            # Chapter overview
    │   ├── high-fidelity-rendering.md  # High-fidelity rendering
    │   ├── human-robot-interaction.md  # Human-robot interaction concepts
    │   └── visual-realism-physical-accuracy.md  # Visual realism vs physical accuracy
    └── sensor-simulation/       # Chapter 3: Sensor Simulation
        ├── index.md            # Chapter overview
        ├── purpose-of-simulated-sensors.md  # Purpose of simulated sensors
        ├── lidar-depth-cameras-imus.md  # LiDAR, depth cameras, IMUs
        └── sensor-data-flow.md  # Sensor data flow to control systems
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