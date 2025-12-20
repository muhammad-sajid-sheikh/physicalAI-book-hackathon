# Quickstart: Module 3 — The AI-Robot Brain (NVIDIA Isaac™)

## Setup Instructions

1. Ensure you have Node.js 18+ installed
2. Install Docusaurus globally: `npm install -g @docusaurus/core@latest`
3. Navigate to the project root directory (physicalAI-book/)
4. Install dependencies: `npm install`
5. Ensure you have ROS 2 Humble Hawksbill installed
6. Install NVIDIA Isaac Sim 2023.1+ (requires NVIDIA GPU with RTX support)
7. Install Isaac ROS components
8. Install Navigation2 (Nav2) packages

## Building the Documentation

1. To start the development server: `npm run start`
2. To build the static site: `npm run build`
3. To serve the built site locally: `npm run serve`

## Adding New Content

1. Create new MD files in the `docs/module-3/` directory
2. Update `sidebars.js` to include the new content in navigation
3. Verify the content renders correctly by running `npm run start`

## Module 3 Structure

Module 3 consists of:
- docs/module-3/index.md - Module overview page
- docs/module-3/chapter-3-1.md - Isaac Sim & Synthetic Data
- docs/module-3/chapter-3-2.md - Isaac ROS Perception
- docs/module-3/chapter-3-3.md - Nav2 for Humanoid Navigation

## Isaac Sim Setup

### Isaac Sim Installation
1. Download Isaac Sim from NVIDIA Developer website
2. Verify your system meets the requirements (RTX GPU, CUDA support)
3. Install Isaac Sim to the default location
4. Launch Isaac Sim and verify the basic scenes work

### Synthetic Dataset Generation
1. Open Isaac Sim and load your scene
2. Configure the synthetic data generation pipeline
3. Set parameters for data collection (image resolution, annotation types, etc.)
4. Run the simulation and collect data

## Isaac ROS Perception

### Environment Setup
1. Source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
2. Source Isaac ROS environment: `source /opt/isaac_ros/setup.sh`
3. Launch Isaac ROS perception nodes: `ros2 launch isaac_ros_apriltag april_tag.launch.py`

### Hardware Acceleration
1. Verify GPU is detected: `nvidia-smi`
2. Launch perception pipelines with GPU acceleration enabled
3. Monitor performance metrics to confirm acceleration benefits

## Navigation2 (Nav2) for Humanoid Robots

### Nav2 Installation
1. Install Nav2 packages: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
2. Source ROS 2 environment: `source /opt/ros/humble/setup.bash`
3. Test Nav2 with a simple example: `ros2 launch nav2_bringup tb3_simulation_launch.py`

### Humanoid Navigation Configuration
1. Create a humanoid-specific Nav2 configuration
2. Adjust parameters for humanoid kinematics (step height, clearance, etc.)
3. Configure planners for bipedal locomotion patterns

## Validation Steps

1. Run `npm run build` to ensure the documentation site builds without errors
2. Check that all navigation links work correctly
3. Verify that new content appears in the sidebar
4. Test responsive behavior on different screen sizes
5. Validate that Isaac Sim examples work as described in the documentation
6. Confirm all exercises are completeable and solutions are accurate
7. Verify Isaac ROS perception pipelines demonstrate hardware acceleration benefits
8. Test Nav2 configuration with humanoid-specific parameters