---
title: Cognitive Planning with LLMs
sidebar_position: 3
---

# Cognitive Planning with LLMs

This chapter explains how large language models can be used for task planning in robotics, enabling humanoid robots to understand complex natural language instructions and translate them into executable action sequences.

## Learning Objectives

After completing this chapter, students will be able to:
- Design cognitive planning systems using LLMs for robotic tasks
- Translate natural language goals into ROS 2 action sequences
- Implement safety and validation checks for LLM-driven actions
- Handle multi-step task decomposition using language models

## Prerequisites

Students should have:
- Understanding of large language models and their capabilities
- Experience with ROS 2 action sequences
- Basic knowledge of natural language processing

## Introduction to Cognitive Planning in Robotics

Cognitive planning in robotics refers to the ability of robots to understand high-level goals expressed in natural language and decompose them into executable action sequences. This capability enables more natural human-robot interaction by allowing users to express their intentions in everyday language rather than specific robot commands.

Large Language Models (LLMs) such as GPT, Claude, and open-source alternatives like Llama have shown remarkable capabilities in understanding and reasoning about complex tasks. When integrated with robotic systems, LLMs can serve as a cognitive layer that bridges natural language instructions and robotic action execution.

### Key Components of LLM-Based Cognitive Planning

1. **Natural Language Understanding**: Interpreting user goals and instructions
2. **Task Decomposition**: Breaking complex tasks into manageable subtasks
3. **Action Sequencing**: Ordering actions based on dependencies and constraints
4. **Environment Modeling**: Understanding the current state and context
5. **Safety Validation**: Ensuring planned actions are safe and appropriate

## How LLMs Enable Task Planning in Robotics

LLMs bring several advantages to robotic task planning:

### 1. Natural Language Interpretation
LLMs can understand complex, ambiguous, or context-dependent instructions that would be difficult to parse with traditional rule-based systems.

**Example**: "Could you please go to the kitchen, find the red cup, and bring it to me? Oh, and if the red cup isn't there, bring the blue one instead."

### 2. Commonsense Reasoning
LLMs possess built-in knowledge about the world that can inform task planning decisions.

**Example**: Understanding that "the kitchen" likely contains cups, or that "bringing" something implies grasping and transporting it.

### 3. Flexible Task Decomposition
LLMs can adapt their planning approach based on context, robot capabilities, and environmental constraints.

## Architecture for LLM-Based Cognitive Planning

The integration of LLMs with robotic systems typically follows this architecture:

```
User Instruction → LLM Cognitive Planner → Task Decomposition → Action Sequences → ROS 2 Execution
                        ↓
                   Environment Context ← → State Monitoring
```

### System Components

1. **Instruction Interface**: Receives natural language goals from users
2. **LLM Cognitive Planner**: Processes instructions and generates task plans
3. **Task Manager**: Orchestrates execution of multi-step tasks
4. **Action Executor**: Interfaces with ROS 2 for low-level execution
5. **State Monitor**: Tracks robot and environment state
6. **Safety Validator**: Ensures actions meet safety constraints

## Implementation Approaches

### Approach 1: Direct LLM Integration
In this approach, the LLM directly generates action sequences or provides high-level guidance that's translated to robot actions.

**Advantages**:
- Simple integration
- Leverages LLM's reasoning capabilities directly

**Disadvantages**:
- Less control over execution
- Potential safety concerns

### Approach 2: LLM-Guided Planning
The LLM provides task decomposition and high-level planning, while traditional planners handle low-level execution.

**Advantages**:
- Better safety and control
- Leverages both LLM reasoning and traditional planning

**Disadvantages**:
- More complex architecture
- Requires more integration work

### Approach 3: Retrieval-Augmented Generation (RAG)
The LLM is augmented with specific robot capabilities, environment maps, and safety constraints.

**Advantages**:
- Context-aware planning
- Better adherence to constraints

**Disadvantages**:
- Requires knowledge base maintenance
- More complex setup

## Integration with Robotic Action Sequences

LLMs can be integrated with ROS 2 action sequences in several ways:

### 1. Action Generation
The LLM generates specific action goals for ROS 2 action servers:

```python
def llm_to_ros_action(llm_output):
    """Convert LLM output to ROS 2 action goals"""
    if llm_output['action'] == 'navigate':
        goal = NavigateToPose.Goal()
        goal.pose = pose_from_location(llm_output['destination'])
        return 'navigate_to_pose', goal
    elif llm_output['action'] == 'manipulate':
        goal = MoveGroup.Goal()
        goal.target = object_to_manipulate(llm_output['object'])
        return 'move_group', goal
```

### 2. Task Decomposition
The LLM breaks down complex tasks into sequences of actions:

```
Input: "Go to kitchen, get red cup, bring to table"
Output:
1. Navigate to kitchen
2. Detect red cup
3. Grasp red cup
4. Navigate to table
5. Place red cup on table
```

### 3. Context-Aware Planning
The LLM considers current state and environment context:

```python
def contextual_planning(user_goal, robot_state, environment_map):
    """Generate plan considering current context"""
    context = {
        'robot_position': robot_state.position,
        'available_objects': environment_map.objects,
        'robot_capabilities': robot_state.capabilities,
        'user_goal': user_goal
    }

    plan = llm.generate_plan(context)
    return plan
```

## Safety and Validation Considerations

LLM-based cognitive planning systems must implement robust safety checks:

### 1. Pre-execution Validation
- Verify actions are within robot capabilities
- Check for environmental safety
- Validate object properties and locations

### 2. Runtime Monitoring
- Monitor execution progress
- Detect and handle failures
- Adjust plans as needed

### 3. Safety Constraints
- Physical safety boundaries
- Operational constraints
- Ethical considerations

## Multi-Step Task Decomposition Techniques

One of the key strengths of LLMs in robotic cognitive planning is their ability to decompose complex tasks into manageable sequences. This section explores techniques for effective multi-step task decomposition.

### Hierarchical Task Networks (HTN)

HTN planning decomposes high-level tasks into sequences of lower-level tasks. LLMs can be used to generate these hierarchical decompositions:

```python
class HTNPlanner:
    def __init__(self, llm_client):
        self.llm = llm_client

    def decompose_task(self, high_level_task, context):
        """Decompose a high-level task into subtasks using LLM"""
        prompt = f"""
        Decompose the following task into executable subtasks:
        Task: {high_level_task}
        Context: {context}

        Provide the decomposition as a list of subtasks in order of execution.
        Each subtask should be specific and executable by a robot.
        """

        response = self.llm.generate(prompt)
        return self.parse_subtasks(response)

    def parse_subtasks(self, llm_response):
        """Parse LLM response into structured subtask list"""
        # Implementation to convert LLM text response to structured subtasks
        subtasks = []
        # Parse the response and create subtask objects
        return subtasks
```

### Dependency-Aware Planning

Complex tasks often have dependencies between subtasks. The LLM can identify these dependencies:

```python
class DependencyAwarePlanner:
    def __init__(self, llm_client):
        self.llm = llm_client

    def analyze_dependencies(self, subtasks):
        """Analyze dependencies between subtasks"""
        prompt = f"""
        Analyze the following subtasks and identify dependencies:
        Subtasks: {subtasks}

        Identify which subtasks must be completed before others can start.
        Return a dependency graph showing the relationships.
        """

        response = self.llm.generate(prompt)
        return self.parse_dependencies(response)
```

### Example: Complex Task Decomposition

Consider the instruction: "Please go to the kitchen, find the red cup, bring it to the table, and wait for me there."

The LLM-based cognitive planner would decompose this into:

1. **Navigation Phase**:
   - Navigate to kitchen
   - Localize robot in kitchen environment

2. **Perception Phase**:
   - Activate object detection
   - Identify red cup in environment
   - Confirm object properties

3. **Manipulation Phase**:
   - Plan grasp trajectory
   - Execute grasp action
   - Verify successful grasp

4. **Transport Phase**:
   - Navigate to table
   - Position for placement

5. **Placement Phase**:
   - Plan placement location
   - Execute placement action

6. **Wait Phase**:
   - Enter waiting state
   - Monitor for user presence

## Safety and Validation in LLM-Driven Actions

LLM-based planning systems require robust safety validation to ensure safe robot behavior:

### Safety Validation Pipeline

```python
class SafetyValidator:
    def __init__(self, robot_capabilities, environment_map):
        self.capabilities = robot_capabilities
        self.env_map = environment_map

    def validate_plan(self, plan):
        """Validate a plan for safety before execution"""
        for step in plan:
            if not self.validate_step(step):
                return False, f"Step {step} failed safety validation"
        return True, "Plan is safe for execution"

    def validate_step(self, step):
        """Validate individual step for safety"""
        # Check if action is within robot capabilities
        if not self.is_action_feasible(step):
            return False

        # Check environmental safety
        if not self.is_environment_safe(step):
            return False

        # Check for potential hazards
        if self.has_hazards(step):
            return False

        return True
```

### Runtime Safety Monitoring

```python
class RuntimeSafetyMonitor:
    def __init__(self, safety_validator):
        self.validator = safety_validator
        self.current_plan = None

    def monitor_execution(self, current_action, robot_state, environment_state):
        """Monitor execution and validate safety in real-time"""
        # Check if current action is still safe given new state information
        is_safe = self.validator.validate_action_with_state(
            current_action, robot_state, environment_state
        )

        if not is_safe:
            return self.handle_safety_violation(current_action)

        return True

    def handle_safety_violation(self, action):
        """Handle safety violations during execution"""
        # Stop current action
        # Generate alternative plan if possible
        # Alert user to the safety issue
        return "SAFETY_VIOLATION_HANDLED"
```

## Practical Exercise: Implementing an LLM-Based Cognitive Planner

In this exercise, students will implement a basic cognitive planning system that uses an LLM to translate natural language goals into ROS 2 action sequences.

### Exercise Requirements:
1. Integrate with an LLM API (e.g., OpenAI, Anthropic, or open-source alternative)
2. Implement task decomposition for multi-step instructions
3. Interface with ROS 2 action servers for execution
4. Include safety validation checks

### Implementation Steps:

1. **Setup LLM Integration**:
   - Configure API access to your chosen LLM
   - Implement prompt engineering for task planning
   - Handle LLM responses and parse structured output

2. **Develop Task Decomposition Logic**:
   - Create functions to break down complex tasks
   - Implement dependency analysis
   - Handle context-aware planning

3. **Connect to ROS 2 Actions**:
   - Map LLM outputs to ROS 2 action goals
   - Implement action execution and monitoring
   - Handle action feedback and results

4. **Add Safety Validation**:
   - Implement capability checks
   - Add environmental safety validation
   - Include runtime monitoring

### Sample Implementation Structure:

```python
class LLMBasedCognitivePlanner:
    def __init__(self, llm_client, action_clients):
        self.llm = llm_client
        self.action_clients = action_clients
        self.safety_validator = SafetyValidator()

    async def plan_and_execute(self, natural_language_goal):
        """Plan and execute a goal expressed in natural language"""
        # 1. Decompose the goal using LLM
        subtasks = await self.decompose_goal(natural_language_goal)

        # 2. Validate the plan for safety
        is_safe, validation_msg = self.safety_validator.validate_plan(subtasks)
        if not is_safe:
            return f"Plan rejected: {validation_msg}"

        # 3. Execute the plan step by step
        for subtask in subtasks:
            result = await self.execute_subtask(subtask)
            if not result.success:
                return f"Execution failed at step: {subtask}"

        return "Goal completed successfully"

    async def decompose_goal(self, goal):
        """Use LLM to decompose high-level goal into subtasks"""
        # Implementation using LLM for task decomposition
        pass

    async def execute_subtask(self, subtask):
        """Execute a single subtask"""
        # Map subtask to ROS 2 action and execute
        pass
```

## Sources & Further Reading

1. Zhu, Y., et al. (2022). "Vision-Language Models Enable Zero-Shot Prediction of Robot Actions." Conference on Robot Learning.
2. Brohan, A., et al. (2022). "RT-1: Robotics Transformer for Real-World Control at Scale." arXiv preprint.
3. Huang, S., et al. (2022). "Language Models as Zero-Shot Planners: Extracting Actionable Knowledge for Embodied Agents." International Conference on Machine Learning.
4. Ahn, M., et al. (2022). "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances." Conference on Robot Learning.
5. Chen, W., et al. (2021). "Learning Transferable Visual Models From Natural Language Supervision." International Conference on Machine Learning.
6. Kaelbling, L.P., et al. (1998). "Planning and Acting in Partially Observable Stochastic Domains." Artificial Intelligence Journal.
7. Fox, M., et al. (2003). "PDDL2.1: An Extension to PDDL for Expressing Temporal Planning Domains." Journal of Artificial Intelligence Research.