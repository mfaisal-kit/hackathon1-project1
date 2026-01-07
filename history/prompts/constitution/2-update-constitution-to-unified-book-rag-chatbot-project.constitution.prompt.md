---
id: 2
title: update-constitution-to-unified-book-rag-chatbot-project
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: pcs
command: /sp.constitution
labels: [constitution, ai, robotics, chatbot, book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - null
---

## Prompt

```text
Project: Unified Book + RAG Chatbot on Physical AI & Humanoid Robotics

Core Principles:

Technical accuracy (verified against official ROS 2, Gazebo, Unity, NVIDIA Isaac docs)

Clarity for students in AI/Robotics

Modular, maintainable structure for Docusaurus

Code correctness and testability

RAG-grounded responses only (no hallucinations)

Key Standards:

All definitions, claims, and technical steps must be sourced from primary documentation

Style: concise, instructional, engineering-focused

Include code examples (Python, ROS 2, FastAPI) fully validated

RAG chatbot must answer only from book content or user-selected text

Include diagrams, architecture flows, and deployment steps

Book Requirements:

Built with Spec-Kit Plus + Claude Code

Deployed via GitHub Pages

Content covers:

ROS 2 nodes, topics, services, URDF

Gazebo & Unity simulation

NVIDIA Isaac Sim, VSLAM, Nav2

Vision-Language-Action robotics

Full humanoid robotics capstone

Chatbot Requirements:

Powered by OpenAI Agents / ChatKit SDK
```

## Response snapshot

Updated the project constitution to reflect the "Unified Book + RAG Chatbot on Physical AI & Humanoid Robotics" project with the specified core principles. The constitution now includes:

- Five core principles: Technical Accuracy, Clarity for AI/Robotics Students, Modularity and Maintainability, Code Correctness and Testability, and RAG-Grounded Responses
- Key Standards and Requirements section
- Book and Chatbot Specifics section with detailed requirements
- Updated template files to align with the new constitution principles:
  - Updated plan-template.md to include constitution check section
  - Updated spec-template.md to include constitution-aligned requirements
  - Updated tasks-template.md to include constitution compliance tasks

## Outcome

- ✅ Impact: Updated project constitution with 5 core principles and aligned templates
- 🧪 Tests: No tests required for constitution update
- 📁 Files: 4 files updated to align with new constitution
- 🔁 Next prompts: Templates now enforce constitution principles during planning
- 🧠 Reflection: Constitution provides clear guidance for AI/Robotics education project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All templates updated correctly
- Prompt variant (if applicable): Standard constitution update
- Next experiment (smallest change to try): Implement first feature following new constitution