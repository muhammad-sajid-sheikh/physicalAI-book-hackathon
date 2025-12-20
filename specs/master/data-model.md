# Data Model: Module 2 — The Digital Twin (Gazebo & Unity)

## Module Entity
- **Name**: Module
- **Fields**:
  - id (string): Unique identifier for the module (e.g., "module-2")
  - title (string): Display title of the module (e.g., "The Digital Twin (Gazebo & Unity)")
  - description (string): Brief description of the module
  - chapters (array): List of chapter entities belonging to this module

## Chapter Entity
- **Name**: Chapter
- **Fields**:
  - id (string): Unique identifier for the chapter (e.g., "chapter-2-1")
  - title (string): Display title of the chapter (e.g., "Gazebo Physics Simulation")
  - module_id (string): Reference to the parent module
  - content (string): Path to the content file
  - word_count (number): Expected word count (1,200-3,000)
  - learning_outcomes (array): List of learning outcomes for the chapter
  - exercises_count (number): Number of exercises (minimum 2)
  - references_count (number): Number of references (minimum 3)

## Simulation Environment Entity
- **Name**: SimulationEnvironment
- **Fields**:
  - id (string): Unique identifier for the environment
  - name (string): Name of the environment (e.g., "Gazebo World")
  - description (string): Description of the environment
  - physics_properties (object): Properties like gravity, friction, etc.
  - robot_models (array): List of robot models that can be spawned in this environment

## Robot Model Entity
- **Name**: RobotModel
- **Fields**:
  - id (string): Unique identifier for the robot model
  - urdf_path (string): Path to the URDF file
  - name (string): Name of the robot
  - joints_count (number): Number of joints in the robot
  - sensors (array): List of sensors attached to the robot
  - physical_properties (object): Mass, inertia, collision properties

## Sensor Entity
- **Name**: Sensor
- **Fields**:
  - id (string): Unique identifier for the sensor
  - type (string): Type of sensor (e.g., "LiDAR", "Camera", "IMU")
  - topic_name (string): ROS topic where sensor data is published
  - update_rate (number): Rate at which sensor publishes data (Hz)
  - robot_model_id (string): Reference to the robot model this sensor is attached to

## Visualization State Entity
- **Name**: VisualizationState
- **Fields**:
  - id (string): Unique identifier for the visualization state
  - timestamp (datetime): When this state was captured
  - robot_position (object): Position coordinates in 3D space
  - robot_orientation (object): Orientation as quaternion
  - joint_states (object): Mapping of joint names to their angles
  - source_simulation (string): Reference to the simulation environment

## Relationships
- A Module contains multiple Chapters (1-to-many)
- Each Chapter belongs to exactly one Module
- A SimulationEnvironment can have multiple RobotModels (1-to-many)
- A RobotModel can have multiple Sensors (1-to-many)
- A VisualizationState references both a SimulationEnvironment and RobotModel (many-to-one)