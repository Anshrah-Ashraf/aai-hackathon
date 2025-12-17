# Data Model: Digital Twin Gazebo Unity Module

**Feature**: 2-digital-twin-gazebo-unity
**Date**: 2025-12-17
**Status**: Complete

## Overview

This document defines the conceptual data model for the Digital Twin educational module. Since this is primarily a documentation project, the "data model" refers to the organizational structure and content entities that will be used in the Docusaurus implementation.

## Content Entities

### Chapter
- **name**: String (e.g., "Physics Simulation with Gazebo", "Environment & Interaction in Unity", "Sensor Simulation")
- **description**: String (brief overview of the chapter's content)
- **sections**: Array of Section entities
- **learning_objectives**: Array of String (what students will learn)
- **prerequisites**: Array of String (what knowledge is required)
- **duration**: Number (estimated time to complete in minutes)

### Section
- **title**: String (e.g., "Role of Digital Twins", "Gravity, Collisions, Physics Engines")
- **content_type**: Enum (text, code_example, diagram, exercise, summary)
- **content**: String (the actual content in Markdown format)
- **dependencies**: Array of Section entities (what needs to be understood first)
- **related_topics**: Array of String (cross-references to other sections)

### Educational Resource
- **type**: Enum (tutorial, example, exercise, reference, glossary)
- **title**: String (descriptive name)
- **content**: String (the resource content in Markdown)
- **target_audience**: Enum (beginner, intermediate, advanced)
- **related_chapters**: Array of Chapter entities

### Code Example
- **language**: String (e.g., "python", "xml", "bash")
- **title**: String (descriptive name)
- **description**: String (what the example demonstrates)
- **code**: String (the actual code snippet)
- **explanation**: String (step-by-step explanation)
- **related_concepts**: Array of String (which concepts this example illustrates)

## Content Relationships

### Chapter Relationships
- Each Chapter contains multiple Sections
- Sections within a Chapter have sequential dependencies
- Chapters may reference content in other Chapters (cross-chapter references)

### Section Relationships
- Sections may include multiple Educational Resources
- Sections may include multiple Code Examples
- Sections may link to related Sections across Chapters

### Educational Resource Relationships
- Resources are categorized by type and difficulty level
- Resources are linked to specific learning objectives
- Resources may be referenced by multiple Sections

## Validation Rules

### Chapter Validation
- Each Chapter must have at least one Section
- Each Chapter must have defined learning objectives
- Chapter titles must be unique within the module
- Chapter duration estimates must be reasonable (between 30-120 minutes)

### Section Validation
- Each Section must have a unique title within its Chapter
- Section content must be in valid Markdown format
- All dependencies must be satisfied (no circular dependencies)
- Section content must align with the Chapter's learning objectives

### Educational Resource Validation
- Resource types must be from the defined enum
- Target audience levels must be appropriate for the content
- All referenced concepts must exist in the module
- Resources must be linked to at least one Section

## State Transitions

### Content Creation Workflow
1. **Draft**: Content is being created and reviewed internally
2. **Review**: Content is ready for review by subject matter experts
3. **Approved**: Content has been approved for publication
4. **Published**: Content is live in the Docusaurus site

### Student Progress Tracking (Conceptual)
1. **Not Started**: Student has not begun the chapter/section
2. **In Progress**: Student is currently working through the content
3. **Completed**: Student has finished the content
4. **Mastered**: Student has demonstrated understanding through exercises

## Content Metadata

### For Each Content Item
- **created_date**: ISO 8601 date string
- **last_modified**: ISO 8601 date string
- **author**: String (name of content creator)
- **reviewers**: Array of String (names of reviewers)
- **version**: Semantic version string
- **spec_reference**: String (reference to the original specification requirement)