# Data Model: Module 3 — The AI-Robot Brain (NVIDIA Isaac™)

## Module Entity
- **Name**: Module
- **Fields**:
  - id (string): Unique identifier for the module (e.g., "module-3")
  - title (string): Display title of the module (e.g., "The AI-Robot Brain (NVIDIA Isaac™)")
  - description (string): Brief description of the module
  - chapters (array): List of chapter entities belonging to this module

## Chapter Entity
- **Name**: Chapter
- **Fields**:
  - id (string): Unique identifier for the chapter (e.g., "chapter-3-1")
  - title (string): Display title of the chapter (e.g., "Isaac Sim & Synthetic Data")
  - module_id (string): Reference to the parent module
  - content (string): Path to the content file
  - word_count (number): Expected word count (1,200-3,000)
  - learning_outcomes (array): List of learning outcomes for the chapter
  - exercises_count (number): Number of exercises (minimum 2)
  - references_count (number): Number of references (minimum 3)

## Isaac Sim Environment Entity
- **Name**: IsaacSimEnvironment
- **Fields**:
  - id (string): Unique identifier for the environment
  - name (string): Name of the environment (e.g., "Isaac Sim Scene")
  - description (string): Description of the environment
  - assets (array): List of 3D assets used in the simulation
  - lighting_config (object): Lighting and shadow properties
  - physics_properties (object): Properties like gravity, friction, etc.
  - robot_models (array): List of robot models that can be simulated in this environment

## Synthetic Dataset Entity
- **Name**: SyntheticDataset
- **Fields**:
  - id (string): Unique identifier for the dataset
  - name (string): Name of the dataset
  - description (string): Description of the dataset contents
  - size (number): Approximate size of the dataset
  - data_types (array): Types of data included (e.g., "images", "LiDAR", "ground_truth")
  - generation_environment_id (string): Reference to the Isaac Sim environment used for generation
  - generation_parameters (object): Parameters used for dataset generation

## Perception Pipeline Entity
- **Name**: PerceptionPipeline
- **Fields**:
  - id (string): Unique identifier for the pipeline
  - name (string): Name of the pipeline (e.g., "VSLAM Pipeline")
  - description (string): Description of the pipeline functionality
  - components (array): List of components in the pipeline (e.g., "feature_detector", "matcher", "optimizer")
  - hardware_acceleration (boolean): Whether the pipeline uses hardware acceleration
  - performance_metrics (object): Performance measurements (FPS, latency, etc.)

## Navigation Waypoint Entity
- **Name**: NavigationWaypoint
- **Fields**:
  - id (string): Unique identifier for the waypoint
  - x (number): X coordinate in 3D space
  - y (number): Y coordinate in 3D space
  - z (number): Z coordinate in 3D space
  - orientation (object): Quaternion representing orientation (x, y, z, w)
  - frame_id (string): Reference frame for the coordinates
  - navigation_goal_id (string): Reference to the navigation goal this waypoint belongs to

## Navigation Goal Entity
- **Name**: NavigationGoal
- **Fields**:
  - id (string): Unique identifier for the navigation goal
  - waypoints (array): Ordered list of waypoints for the navigation path
  - robot_model_id (string): Reference to the robot model for which this goal is planned
  - environment_constraints (object): Constraints specific to the environment
  - success_criteria (array): Conditions that define goal achievement

## Relationships
- A Module contains multiple Chapters (1-to-many)
- Each Chapter belongs to exactly one Module
- An Isaac Sim Environment can generate multiple Synthetic Datasets (1-to-many)
- A Perception Pipeline can be used in multiple Robot Models (many-to-many)
- A Navigation Goal contains multiple Waypoints (1-to-many)
- A Navigation Waypoint belongs to exactly one Navigation Goal