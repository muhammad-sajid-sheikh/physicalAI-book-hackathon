---
sidebar_position: 4
title: 'Chapter 3.3 — Nav2 for Humanoid Navigation'
---

# Chapter 3.3: Nav2 for Humanoid Navigation

## Introduction

Navigation2 (Nav2) is the state-of-the-art navigation framework for ROS 2, providing a complete solution for path planning, path execution, and recovery behaviors. While originally designed for wheeled robots, Nav2 can be adapted for humanoid robots with careful consideration of their unique kinematic and dynamic properties. This chapter explores how to configure and customize Nav2 for humanoid robot navigation, taking into account the specific challenges and requirements of bipedal locomotion systems.

## Learning Objectives

By the end of this chapter, you should be able to:
1. Understand the Nav2 architecture and its components
2. Configure Nav2 for humanoid robot kinematics and constraints
3. Implement path planning algorithms suitable for humanoid robots
4. Customize navigation behaviors for bipedal locomotion
5. Integrate Nav2 with humanoid-specific controllers and perception systems

## What is Navigation2 (Nav2)?

Navigation2 is the next-generation navigation framework for ROS 2, designed to be more robust, performant, and feature-rich than its predecessor. Key features include:

- **Behavior Trees**: Flexible, composable navigation behaviors
- **Plugin Architecture**: Extensible components for custom algorithms
- **Improved Recovery**: Sophisticated recovery behaviors for complex scenarios
- **Better Performance**: Optimized for real-time applications
- **Enhanced Safety**: Comprehensive safety mechanisms and collision avoidance

### Core Components

1. **Navigation Server**: Main orchestrator for navigation tasks
2. **Planners Server**: Global and local path planning
3. **Controller Server**: Path following and trajectory generation
4. **Recovery Server**: Recovery behavior management
5. **BT Navigator**: Behavior tree-based navigation execution

## Nav2 Architecture

### System Overview

Nav2 follows a client-server architecture with multiple specialized servers:

```
[Navigation Server]
    ├── [Planners Server] - Global/Local planners
    ├── [Controller Server] - Path following
    ├── [Recovery Server] - Recovery behaviors
    └── [BT Navigator] - Behavior tree execution
```

### Behavior Tree Navigation

Nav2 uses behavior trees for navigation orchestration:

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <PipelineSequence name="NavigateWithReplanning">
            <RateController hz="1.0">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            </RateController>
            <ReactiveFallback name="FollowPathOrRecover">
                <FollowPath path="{path}" controller_id="FollowPath"/>
                <ReactiveSequence>
                    <RecoveryNode recovery_behavior_id="spin"/>
                    <RecoveryNode recovery_behavior_id="backup"/>
                    <RecoveryNode recovery_behavior_id="wait"/>
                </ReactiveSequence>
            </ReactiveFallback>
        </PipelineSequence>
    </BehaviorTree>
</root>
```

## Humanoid Robot Kinematics Considerations

### Differences from Wheeled Robots

Humanoid robots present unique challenges for navigation:

1. **Bipedal Locomotion**: Requires balance maintenance during movement
2. **Limited Turning Radius**: Cannot turn in place like differential drives
3. **Step Constraints**: Must navigate stairs, curbs, and uneven terrain
4. **Stability Requirements**: Need to maintain center of mass within support polygon
5. **Foot Placement**: Requires precise footstep planning

### Kinematic Modeling

Humanoid robots have complex kinematic chains:

```python
# Example: Humanoid kinematic model
class HumanoidKinematics:
    def __init__(self, robot_description):
        self.torso_height = 0.8  # meters
        self.leg_length = 0.9   # meters
        self.step_width = 0.2   # meters (lateral step distance)
        self.step_length = 0.3  # meters (forward step distance)
        self.turn_radius = 0.4  # meters (minimum turning radius)

    def is_valid_footstep(self, position, orientation):
        """Check if a footstep is kinematically feasible"""
        # Check step length and width constraints
        # Check balance constraints
        # Check obstacle clearance
        pass

    def generate_feasible_trajectory(self, path):
        """Generate a humanoid-feasible trajectory from a path"""
        # Convert geometric path to footstep sequence
        # Ensure balance during transitions
        # Generate smooth COM trajectory
        pass
```

## Nav2 Configuration for Humanoid Robots

### Parameter File Structure

A typical Nav2 configuration for humanoid robots:

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "package://nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    default_nav_to_pose_bt_xml: "package://nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_smooth_path_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_drive_on_heading_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_have_feedback_condition_bt_node
      - nav2_have_global_plan_condition_bt_node
      - nav2_have_local_plan_condition_bt_node
      - nav2_cost_inspection_layer_bt_node
      - nav2_inflation_layer_bt_node
      - nav2_dynamic_obstacle_condition_bt_node
      - nav2_orientation_filter_bt_node
      - nav2_is_path_valid_condition_bt_node

bt_navigator_rclcpp_node:
  ros__parameters:
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      # Humanoid-specific parameters
      time_steps: 20
      control_horizon: 10
      model_dt: 0.1
      xy_goal_tolerance: 0.3  # Larger tolerance for humanoid balance
      yaw_goal_tolerance: 0.3
      state_bounds_x: [-0.5, 0.5]  # Limited step speeds
      state_bounds_y: [-0.2, 0.2]
      state_bounds_theta: [-0.5, 0.5]
      ctrl_thresh_xy: 0.01
      ctrl_thresh_theta: 0.02
      mass: 50.0  # Humanoid robot mass
      friction_coeff: 0.7
      power_penalty: 10.0
      reference_scale: 1.0
      goal_ang_scale: 1.0
      goal_lin_scale: 5.0
      xy_goal_scale: 5.0
      ang_goal_scale: 3.0
      obstacle_scale: 50.0
      trap_scale: 1.0
      heading_scale: 0.0
      forward_preference_scale: 0.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      use_roll_pitch: true  # For humanoid with 6DOF poses
      use_dijkstra: false
      use_grid_path: false
      allow_unknown: true
      lethal_cost_threshold: 100
      # Humanoid-specific parameters
      robot_radius: 0.4  # Larger radius for humanoid footprint
      resolution: 0.05
      inflation_radius: 0.5  # Larger inflation for humanoid safety
      observation_sources: scan
      scan:
        topic: /laser_scan
        max_obstacle_height: 2.0  # Up to humanoid height
        clearing: true
        marking: true
        data_type: LaserScan
        raytrace_max_range: 3.0
        raytrace_min_range: 0.0
        obstacle_max_range: 2.5
        obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_roll_pitch: true
      robot_radius: 0.4
      resolution: 0.1
      track_unknown_space: true
      # Humanoid-specific parameters
      inflation_radius: 0.6  # Larger inflation for humanoid safety
      observation_sources: scan
      scan:
        topic: /laser_scan
        max_obstacle_height: 2.0
        clearing: true
        marking: true
        data_type: LaserScan
        raytrace_max_range: 10.0
        raytrace_min_range: 0.0
        obstacle_max_range: 8.0
        obstacle_min_range: 0.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5  # Larger tolerance for humanoid navigation
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      use_orientation_filtering: true
      orientation_filter_angle: 1.57  # 90 degrees for humanoid turning constraints

## Humanoid Path Planning

### Specialized Planners for Bipedal Robots

Traditional path planners need adaptation for humanoid robots due to their unique kinematics:

1. **Footstep Planning**: Rather than continuous paths, humanoid robots need discrete footstep sequences
2. **Balance Maintenance**: Paths must ensure the robot's center of mass remains within the support polygon
3. **Terrain Traversability**: Must consider step height, slope, and surface stability
4. **Turning Constraints**: Limited ability to turn in place requires path smoothing

### Footstep Planner Integration

```python
from nav2_core.base_global_planner import BaseGlobalPlanner
from geometry_msgs.msg import PoseStamped
import numpy as np

class HumanoidFootstepPlanner(BaseGlobalPlanner):
    def __init__(self):
        super().__init__()
        self.foot_separation = 0.2  # Distance between feet
        self.max_step_length = 0.3  # Maximum forward step
        self.max_step_width = 0.4   # Maximum lateral step
        self.turn_radius = 0.4      # Minimum turning radius

    def create_plan(self, start, goal, robot_radius=0.0):
        """
        Create a footstep plan for humanoid robot
        """
        # Generate initial geometric path
        geometric_path = self.generate_geometric_path(start, goal)

        # Convert to footstep sequence
        footsteps = self.geometric_to_footsteps(geometric_path)

        # Validate footsteps for balance
        valid_footsteps = self.validate_balance(footsteps)

        # Generate COM trajectory
        com_trajectory = self.generate_com_trajectory(valid_footsteps)

        return com_trajectory

    def geometric_to_footsteps(self, path):
        """
        Convert geometric path to sequence of foot placements
        """
        footsteps = []
        left_support = True  # Start with right foot swing

        for i in range(len(path) - 1):
            # Calculate step direction and distance
            dx = path[i+1].pose.position.x - path[i].pose.position.x
            dy = path[i+1].pose.position.y - path[i].pose.position.y
            dist = np.sqrt(dx*dx + dy*dy)

            if dist > self.max_step_length:
                # Need to interpolate to keep steps within limits
                steps_needed = int(np.ceil(dist / self.max_step_length))
                for j in range(steps_needed):
                    step_x = path[i].pose.position.x + (j+1) * dx / steps_needed
                    step_y = path[i].pose.position.y + (j+1) * dy / steps_needed

                    foot_pose = self.calculate_foot_placement(step_x, step_y, left_support)
                    footsteps.append(foot_pose)
                    left_support = not left_support
            else:
                foot_pose = self.calculate_foot_placement(
                    path[i+1].pose.position.x,
                    path[i+1].pose.position.y,
                    left_support
                )
                footsteps.append(foot_pose)
                left_support = not left_support

        return footsteps

    def validate_balance(self, footsteps):
        """
        Validate that footsteps maintain balance
        """
        valid_steps = []
        for i, step in enumerate(footsteps):
            # Check that center of mass projection is within support polygon
            if self.check_balance_at_step(step, footsteps[:i]):
                valid_steps.append(step)

        return valid_steps
```

## Navigation Behaviors for Humanoid Robots

### Custom Recovery Behaviors

Humanoid robots need specialized recovery behaviors:

```xml
<BehaviorTree ID="NavigateWReplanningAndHumanoidRecovery">
    <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        </RateController>
        <ReactiveFallback name="FollowPathOrRecover">
            <FollowPath path="{path}" controller_id="FollowPath"/>
            <ReactiveSequence>
                <RecoveryNode recovery_behavior_id="humanoid_spin"/>
                <RecoveryNode recovery_behavior_id="humanoid_backup"/>
                <RecoveryNode recovery_behavior_id="humanoid_wait"/>
                <RecoveryNode recovery_behavior_id="humanoid_stepaside"/>
            </ReactiveSequence>
        </ReactiveFallback>
    </PipelineSequence>
</BehaviorTree>
```

### Step-Aside Recovery Behavior

For narrow passages where turning isn't possible:

```python
from nav2_core.recovery import Recovery
from geometry_msgs.msg import Twist
import math

class StepAsideRecovery(Recovery):
    def __init__(self):
        super().__init__()
        self.cmd_vel_pub = None
        self.costmap_ros = None
        self.tf_buffer = None
        self.step_distance = 0.3  # meters to step aside

    def run(self, original_command):
        """
        Move robot laterally to clear obstacles
        """
        robot_pose = self.get_robot_pose()

        # Look for clear space to the left and right
        left_clear = self.check_lateral_clearance(robot_pose, math.pi/2)
        right_clear = self.check_lateral_clearance(robot_pose, -math.pi/2)

        if left_clear:
            self.execute_lateral_move(math.pi/2, self.step_distance)
        elif right_clear:
            self.execute_lateral_move(-math.pi/2, self.step_distance)
        else:
            # No clear space, try small forward/backward move
            self.execute_longitudinal_move(0.1)  # Small forward move

    def check_lateral_clearance(self, pose, angle_offset):
        """
        Check if there's clearance in the lateral direction
        """
        test_pose = Pose()
        test_pose.position.x = pose.position.x + 0.3 * math.cos(pose.orientation.z + angle_offset)
        test_pose.position.y = pose.position.y + 0.3 * math.sin(pose.orientation.z + angle_offset)

        # Check costmap at test pose
        cost = self.costmap_ros.getCostmap().getCost(
            int(test_pose.position.x),
            int(test_pose.position.y)
        )

        return cost < 50  # Free or slightly occupied space

    def execute_lateral_move(self, angle, distance):
        """
        Execute a lateral movement
        """
        # For humanoid, this would involve coordinated foot movements
        # This is a simplified representation
        duration = distance / 0.1  # Assuming 0.1 m/s lateral speed
        self.move_with_duration(angle, distance, duration)

## Footstep Controller

A specialized controller for humanoid navigation:

```python
from nav2_core.controller import Controller
from geometry_msgs.msg import Twist, PoseStamped
from builtin_interfaces.msg import Duration
import numpy as np

class HumanoidFootstepController(Controller):
    def __init__(self):
        super().__init__()
        self.foot_separation = 0.2  # meters
        self.step_height = 0.05     # meters
        self.step_duration = 1.0    # seconds per step
        self.balance_margin = 0.1   # safety margin for balance

    def setPlan(self, path):
        """
        Set the plan to follow
        """
        self.path = path
        self.current_waypoint = 0
        self.last_step_time = 0

    def computeVelocityCommands(self, pose, velocity, cmd_vel):
        """
        Compute velocity commands for humanoid robot
        """
        # For humanoid robots, we need to compute footstep trajectories
        # rather than continuous velocity commands

        if self.current_waypoint >= len(self.path.poses):
            # Reached goal
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            return cmd_vel

        # Calculate next footstep
        next_waypoint = self.path.poses[self.current_waypoint]

        # Check if ready for next step
        if self.ready_for_next_step():
            footstep = self.calculate_next_footstep(pose, next_waypoint)
            self.execute_footstep(footstep)
            self.current_waypoint += 1

            # Set command to move to next step
            cmd_vel.linear.x = 0.1  # Conservative walking speed
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = self.calculate_turn(pose, next_waypoint)
        else:
            # Still executing current step
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0

        return cmd_vel

    def calculate_next_footstep(self, current_pose, target_pose):
        """
        Calculate next footstep based on current pose and target
        """
        # Calculate step direction and magnitude
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)

        # Normalize direction vector
        if distance > 0:
            dx /= distance
            dy /= distance

        # Limit step size
        step_size = min(distance, self.max_step_size)

        # Calculate foot placement (alternating feet)
        foot_offset_x = step_size * dx
        foot_offset_y = step_size * dy

        # Add to current position
        foot_pose = PoseStamped()
        foot_pose.header.stamp = self.get_clock().now().to_msg()
        foot_pose.header.frame_id = "map"
        foot_pose.pose.position.x = current_pose.pose.position.x + foot_offset_x
        foot_pose.pose.position.y = current_pose.pose.position.y + foot_offset_y
        foot_pose.pose.position.z = 0.0  # Ground level

        # Orient foot along path direction
        foot_pose.pose.orientation = self.calculate_orientation(dx, dy)

        return foot_pose

    def ready_for_next_step(self):
        """
        Check if robot is ready for next step
        """
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_step_time).nanoseconds / 1e9

        # Ready if enough time has passed since last step
        return elapsed >= self.step_duration
```

## Integration with Humanoid Control Systems

### Walking Pattern Generators

Integration with humanoid walking pattern generators:

```python
class Nav2HumanoidIntegration:
    def __init__(self):
        self.navigation_active = False
        self.walking_pattern_generator = None
        self.balance_controller = None
        self.footstep_planner = HumanoidFootstepPlanner()

        # Navigation-related parameters
        self.nav_goal = None
        self.current_path = None

    def start_navigation(self, goal):
        """
        Start navigation to specified goal
        """
        self.nav_goal = goal
        self.navigation_active = True

        # Plan path using Nav2
        path = self.request_global_path(goal)

        if path:
            # Convert to footstep plan
            footstep_plan = self.footstep_planner.create_plan(
                self.get_current_pose(),
                goal
            )

            # Execute with walking pattern generator
            self.walking_pattern_generator.execute_plan(footstep_plan)

            # Activate balance controller
            self.balance_controller.activate()

    def stop_navigation(self):
        """
        Stop current navigation
        """
        self.navigation_active = False
        self.walking_pattern_generator.stop()
        self.balance_controller.deactivate()

    def handle_obstacles(self, obstacle_msg):
        """
        Handle dynamic obstacles during navigation
        """
        if self.navigation_active:
            # Request replanning
            new_path = self.request_global_path(self.nav_goal)

            if new_path:
                # Update footstep plan
                footstep_plan = self.footstep_planner.create_plan(
                    self.get_current_pose(),
                    self.nav_goal,
                    obstacle_avoidance=True
                )

                # Update walking pattern generator
                self.walking_pattern_generator.update_plan(footstep_plan)

## Simulation and Testing

### Testing in Isaac Sim

Testing Nav2 with humanoid robots in Isaac Sim:

1. **Environment Setup**: Create diverse environments with stairs, ramps, and obstacles
2. **Robot Model**: Use accurate humanoid model with proper kinematics
3. **Sensor Simulation**: Include realistic sensor models (LiDAR, cameras, IMU)
4. **Physics Configuration**: Set appropriate physics parameters for stable walking

### Performance Metrics

Key metrics for evaluating humanoid navigation:

1. **Success Rate**: Percentage of navigation attempts that reach the goal
2. **Path Efficiency**: Ratio of actual path length to optimal path length
3. **Balance Maintenance**: Time spent in balanced vs unbalanced states
4. **Obstacle Avoidance**: Ability to navigate around dynamic obstacles
5. **Computational Efficiency**: CPU/GPU usage during navigation

## Challenges and Solutions

### Balance Preservation

Challenge: Maintaining balance during navigation
Solution:
- Implement zero-moment point (ZMP) based trajectory generation
- Use preview control for smoother footstep transitions
- Integrate with whole-body controllers for balance maintenance

### Terrain Adaptation

Challenge: Navigating uneven terrain and obstacles
Solution:
- Use traversability analysis for footstep planning
- Implement online terrain classification
- Adaptive gait generation based on terrain type

### Real-time Performance

Challenge: Meeting real-time constraints for walking control
Solution:
- Optimize path planning algorithms for humanoid constraints
- Use predictive control for smoother motion
- Implement efficient collision checking algorithms

## Exercises

1. **Nav2 Configuration**: Configure Nav2 for a humanoid robot model with appropriate parameters for kinematic constraints. Test the configuration in simulation with various navigation scenarios.

2. **Footstep Planning**: Implement a basic footstep planner that converts geometric paths to humanoid-appropriate footstep sequences. Validate that the generated steps maintain balance.

3. **Behavior Tree Customization**: Create a custom behavior tree for humanoid navigation that includes specialized recovery behaviors for bipedal robots. Test the behavior tree in simulation with challenging navigation scenarios.

## References

1. ROS 2 Documentation. (2023). "Navigation2 User Guide". https://navigation.ros.org/
2. Sisbot, E. A., et al. (2008). "Planning human-aware motions using a sampling-based cost-space planner". Proceedings of the IEEE International Conference on Robotics and Automation.
3. Winkler, S., et al. (2018). "Humanoid navigation with dynamic obstacle avoidance". International Journal of Humanoid Robotics.

## Summary

Nav2 provides a comprehensive navigation framework that can be adapted for humanoid robots with careful consideration of their unique kinematic and dynamic properties. By customizing planners, controllers, and recovery behaviors for humanoid constraints, we can achieve effective navigation for bipedal robots. The key is understanding the differences between wheeled and humanoid navigation and implementing appropriate solutions for balance preservation, terrain adaptation, and real-time performance. With proper configuration and integration with humanoid control systems, Nav2 enables autonomous navigation capabilities that leverage the full potential of humanoid robot platforms.