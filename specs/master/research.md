# Research: Module 2 — The Digital Twin (Gazebo & Unity)

## Decision: Gazebo Version
**Rationale**: Using Gazebo Harmonic or Garden as these are the current stable versions with good ROS 2 integration and physics capabilities needed for humanoid robot simulation.
**Alternatives considered**: Gazebo Classic (older), Ignition Dome (older), custom physics engine (more complex)

## Decision: Unity Version
**Rationale**: Using Unity 2022.3 LTS as it provides long-term support, stability, and has good ROS integration packages available (ROS# or similar).
**Alternatives considered**: Unity 2023+ (newer but less tested), Unreal Engine (different skill set required), custom visualization (more complex)

## Decision: Module Structure
**Rationale**: Creating a module-2 directory with individual chapter files to maintain clear organization and follow Docusaurus best practices.
**Alternatives considered**: Single monolithic file (harder to manage), different directory naming (less clear)

## Decision: Navigation Approach
**Rationale**: Using Docusaurus sidebars.js for navigation as it's the standard approach and integrates well with the documentation structure.
**Alternatives considered**: Custom navigation component (more complex), flat structure without hierarchy (less organized)

## Decision: Frontmatter Format
**Rationale**: Using standard Docusaurus frontmatter with title, description, and other metadata to ensure proper rendering and SEO.
**Alternatives considered**: Minimal frontmatter (less metadata), custom fields (not standard)

## Decision: Sensor Simulation Approach
**Rationale**: Using Gazebo's built-in sensor plugins for LiDAR, camera, and IMU simulation as they provide realistic physics-based sensor data that matches real hardware characteristics.
**Alternatives considered**: Custom sensor simulation (less realistic), simplified sensor models (less educational value)