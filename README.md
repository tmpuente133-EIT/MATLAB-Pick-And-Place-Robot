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
 **Figure 1 — Robot setup and collision geometry**
 
 <img src= "images/loadrobot.jpg" alt="loadrobot" width="400">

 This section shows how the KUKA iiwa robot is imported and augmented with simplified collision geometries.Custom collision shapes are assigned to the robot links to enable fast and reliable collision checking during planning, independent of visual meshes.

---

### 2) Collision-Safe Pick & Place Logic
 **Figure 2 — IK-based pick and place configuration**
 
<img src= "images/pickpose.jpg" alt="pickpose" width="400">

 This section highlights the inverse kinematics workflow used to compute feasible pick and place configurations. A collision-aware search strategy is used to find the closest valid placement when the user’s requested target is not directly reachable.

---

### 3) RRT Motion Planning & Execution
 **Figure 3 — RRT planning and execution**
 
<img src= "images/Screenshot%202026-02-01%20175812.png" alt="pickpose" width="400">

 This section demonstrates how a Rapidly-exploring Random Tree (RRT) planner is used to generate collision-free joint-space trajectories. Separate plans are generated for the **Home → Pick** and **Pick → Place** motions, which are then concatenated into a single smooth trajectory.

---

## System Demonstration

The video shows the robot executing the full task:
- Moving from home configuration
- Picking one of two objects
- Transporting the object while avoiding obstacles
- Placing the object at the target location

<a href="https://youtu.be/Xx_lmXefTAo" target="_blank">
  <img src="images/robot.png" alt="pickpose" width="400">
</a>

*Click the image to watch the full collision-aware pick-and-place demonstration.*

---

## Tools & Concepts Used

- MATLAB Robotics System Toolbox  
- RigidBodyTree modeling  
- Inverse kinematics  
- Collision checking  
- RRT motion planning  
- Trajectory interpolation and visualization

## Outcome

This project demonstrates a complete, collision-aware robotic manipulation pipeline and illustrates how high-level planning algorithms can be combined with kinematic modeling to produce reliable autonomous behavior in a constrained environment.
