# KUKA iiwa Collision-Aware Pick-and-Place

This project demonstrates a **collision-aware pick-and-place task** using a KUKA iiwa robot model.  
The system integrates **inverse kinematics (IK)**, **environment collision modeling**, and **RRT-based motion planning** to autonomously move objects between platforms while avoiding obstacles.

A short screen recording of the system running is included below, along with highlighted code excerpts that illustrate the core implementation.

---

## Project Overview

The goal of this project was to design a motion-planning pipeline that allows a robotic manipulator to:
1. Identify and reach a pick object
2. Safely transport the object through a cluttered workspace
3. Place the object at a user-defined target location without collisions

Rather than focusing on low-level mechanics, this project emphasizes **planning logic, collision reasoning, and system integration**.

---

## Key Implementation Highlights

### 1) Robot Import & Collision Modeling
 **Screenshot 1 — Robot setup and collision geometry**

> This section shows how the KUKA iiwa robot is imported and augmented with simplified collision geometries.  
> Custom collision shapes are assigned to the robot links to enable fast and reliable collision checking during planning, independent of visual meshes.

**Screenshot includes:**
- Robot import
- Collision object assignment
- Gravity and kinematic setup

---

### 2) Collision-Safe Pick & Place Logic
 **Screenshot 2 — IK-based pick and place configuration**

> This section highlights the inverse kinematics workflow used to compute feasible pick and place configurations.  
> A collision-aware search strategy is used to find the closest valid placement when the user’s requested target is not directly reachable.

**Screenshot includes:**
- Pick pose definition above the object
- User-selected target position
- Collision-checked place configuration search

---

### 3) RRT Motion Planning & Execution
 **Screenshot 3 — RRT planning and execution**

> This section demonstrates how a Rapidly-exploring Random Tree (RRT) planner is used to generate collision-free joint-space trajectories.  
> Separate plans are generated for the **Home → Pick** and **Pick → Place** motions, which are then concatenated into a single smooth trajectory.

**Screenshot includes:**
- RRT planner setup
- Path planning calls
- Trajectory smoothing and execution logic

---

## System Demonstration

 **Screen Recording — Full Pick-and-Place Execution**

The video below shows the robot executing the full task:
- Moving from home configuration
- Picking one of two objects
- Transporting the object while avoiding obstacles
- Placing the object at the target location

> *(Embedded video or link to recording here)*

---

## Tools & Concepts Used

- MATLAB Robotics System Toolbox  
- RigidBodyTree modeling  
- Inverse kinematics  
- Collision checking  
- RRT motion planning  
- Trajectory interpolation and visualization

---

## Outcome

This project demonstrates a complete, collision-aware robotic manipulation pipeline and illustrates how high-level planning algorithms can be combined with kinematic modeling to produce reliable autonomous behavior in a constrained environment.
