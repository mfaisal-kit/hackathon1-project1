---
id: 1
title: Update Project Constitution with AI/Robotics Chatbot Details
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: Unified Book + RAG Chatbot on Physical AI & Humanoid Robotics
branch: master
user: pcs
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- .specify/memory/constitution.md
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

```text
The project constitution has been updated in .specify/memory/constitution.md to version 1.0.0.
It incorporates the core principles, key standards, and specific requirements for the Unified Book + RAG Chatbot on Physical AI & Humanoid Robotics.
Other project templates (.specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md, and command files) are flagged as pending updates to align with the new constitution.
```

## Outcome

- ✅ Impact: Constitution updated, foundational principles defined.
- 🧪 Tests: No specific tests for constitution update.
- 📁 Files: One file modified: .specify/memory/constitution.md.
- 🔁 Next prompts: User might ask to update other templates.
- 🧠 Reflection: Successfully updated constitution based on user input, ensuring all principles and requirements are captured. Noted pending updates for dependent templates.

## Evaluation notes (flywheel)

- Failure modes observed: None observed.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure automatic propagation of constitution changes to dependent templates or prompt user to do so.
