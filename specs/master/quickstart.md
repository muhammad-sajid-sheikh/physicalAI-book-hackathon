# Quickstart: Module 2 — The Digital Twin (Gazebo & Unity)

## Setup Instructions

1. Ensure you have Node.js 18+ installed
2. Install Docusaurus globally: `npm install -g @docusaurus/core@latest`
3. Navigate to the project root directory (physicalAI-book/)
4. Install dependencies: `npm install`
5. Ensure you have ROS 2 Humble Hawksbill installed
6. Install Gazebo Harmonic or Garden
7. Install Unity 2022.3 LTS or later

## Building the Documentation

1. To start the development server: `npm run start`
2. To build the static site: `npm run build`
3. To serve the built site locally: `npm run serve`

## Adding New Content

1. Create new MD files in the `docs/module-2/` directory
2. Update `sidebars.js` to include the new content in navigation
3. Verify the content renders correctly by running `npm run start`

## Module 2 Structure

Module 2 consists of:
- docs/module-2/index.md - Module overview page
- docs/module-2/chapter-2-1.md - Gazebo Physics Simulation
- docs/module-2/chapter-2-2.md - Sensor Simulation
- docs/module-2/chapter-2-3.md - Unity Digital Twin Visualization

## Simulation Setup

### Gazebo Physics Simulation
1. Source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
2. Launch the Gazebo simulation: `ros2 launch <package_name> <launch_file>.py`
3. Verify physics properties (gravity, collisions) are working properly

### Sensor Simulation
1. Launch your robot model with sensor plugins in Gazebo
2. Verify sensor topics are being published: `ros2 topic list | grep /sensor`
3. Check sensor data rates and quality

### Unity Digital Twin Visualization
1. Open the Unity project
2. Configure the ROS connection settings
3. Run the Unity scene to visualize robot state

## Validation Steps

1. Run `npm run build` to ensure the documentation site builds without errors
2. Check that all navigation links work correctly
3. Verify that new content appears in the sidebar
4. Test responsive behavior on different screen sizes
5. Validate that simulation examples work as described in the documentation
6. Confirm all exercises are completeable and solutions are accurate