# Autonomous Pick-and-Place Robot with Obstacle Avoidance

## Overview
This repository documents the design, implementation, and simulation of an autonomous robotic manipulator capable of picking and placing objects in a cluttered environment while avoiding obstacles. The project was developed using MATLAB and the Robotics System Toolbox as part of an advanced robot modeling and control course.

---

## Problem Statement
Industrial and service robots must be able to autonomously manipulate objects in environments that contain obstacles and user-defined goals. The objective of this project was to develop a robotic manipulation system that can safely pick up objects and place them at arbitrary target locations specified by the user, provided those locations lie within the robot’s reachable workspace. The robot must detect unreachable targets, plan collision-free trajectories, and execute reliable pick-and-place motions without human intervention.

---

## Project Objectives
- Import and configure a realistic robot manipulator model
- Create a simulated environment with obstacles and movable objects
- Implement inverse kinematics for grasping and placement
- Plan collision-free trajectories using sampling-based motion planning
- Enable user-defined placement goals with workspace validation
- Visualize and animate the full pick-and-place process

---

## System Description

### Robot Model
- Robot imported using MATLAB’s `loadrobot` function
- Full URDF-based kinematic structure
- Custom collision geometry added to each link

📌 *Add image here:*  
`![Robot model visualization](media/robot_model.png)`

---

### Environment Setup
- Static environment composed of a floor and two platforms
- Obstacles modeled using collision primitives
- Two movable box objects placed on platforms

📌 *Add image here:*  
`![Simulation environment](media/environment.png)`

---

## Motion Planning and Control Architecture

### Inverse Kinematics
- Numerical inverse kinematics used to compute joint configurations
- Pre-grasp, grasp, pre-place, and place poses defined in task space
- Workspace checks implemented to reject unreachable goals

📌 *Relevant code:*  
`src/kinematics/computeIK.m`

---

### Trajectory Planning
- Sampling-based motion planning using `manipulatorRRT`
- Collision checking against environment and objects
- Straight-line interpolation used for precision near grasp and placement

📌 *Relevant code:*  
`src/planning/planPath.m`

---

### Obstacle Avoidance
- All robot motions validated against collision geometry
- Objects treated as obstacles until grasped
- Dynamic update of obstacles during object transport

📌 *Relevant code:*  
`src/planning/checkCollisionWrapper.m`

---

## Pick-and-Place Workflow
1. Plan path from home position to pre-grasp pose
2. Execute controlled approach to grasp the object
3. Attach object to end-effector
4. Plan collision-free path to target platform
5. Place object and retreat safely

📌 *Add animation or GIF here:*  
`![Pick-and-place animation](media/pick_and_place.gif)`

---

## User Interaction
- User specifies target position in Cartesian space
- System validates reachability and collision feasibility
- Clear error messages returned for invalid targets

📌 *Relevant code:*  
`src/interface/userInputHandler.m`

---

## Challenges and Solutions
- **Collision geometry misalignment:** resolved by iterative tuning and frame verification  
- **Unreachable IK targets:** mitigated using workspace checks and goal regions  
- **RRT planning failures:** addressed by adjusting planner parameters and adding intermediate waypoints  
- **Object handling during motion:** solved by dynamically switching object roles between obstacle and payload  

---

## Results
- Successful autonomous pick-and-place of multiple objects
- Reliable obstacle avoidance in cluttered environments
- Robust response to valid and invalid user inputs

📌 *Add results plot or screenshot:*  
`![Final placement result](media/final_result.png)`

---

## Repository Structure
```text
.
├── src/
│   ├── robot/          # Robot import and collision setup
│   ├── environment/    # Environment and object definitions
│   ├── kinematics/     # Inverse kinematics algorithms
│   ├── planning/       # RRT and trajectory planning
│   └── interface/      # User input handling
├── media/              # Images, GIFs, and videos
├── scripts/            # Main execution scripts
└── README.md
