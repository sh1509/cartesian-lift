# Project Plan: Cartesian Motion with Vertical Lift (48-Hour Sprint)

## Overview

This project involves simulating and controlling Cartesian motions of a 6-DOF robotic arm mounted on a vertical prismatic joint (lift) using ROS 2 and Ignition Gazebo. It includes velocity-controlled motion, trajectory generation, and optional compliance control.

__Notes__ \
I will also be working on my current office work, so following this time schedule exactly is not completely possible, but I will try my best.

---

## Environment Setup (Hours 0–2)

### Deliverables:

* [ ] Dockerfile for ROS 2 + Ignition Gazebo
* [ ] Verified URDF model with 6-DOF

### Platform dependencies:

*  Humble/Foxy ROS 2 base image

###
Start time: 25/07 01:00 - 1:30
Resumed: 25/07 09:30

---

## Phase 1: Robot Model & Simulation (Hours 2–8)

### Deliverables:

* [ ] Modified URDF: 6-DOF robot + vertical prismatic joint
* [ ] Simulation world with robot spawned in Ignition Gazebo
* [ ] Basic Cartesian motion at fixed Z using trapezoidal profile

### Tools:

* xacro, RViz, Ignition Gazebo, `ros2 launch`

---

## Phase 2: Z-Lift Control (Hours 8–16)

### Deliverables:

* [ ] Prismatic joint integrated into motion planning
* [ ] Cartesian motion on 2D plane (XY) at different Z heights
* [ ] Trajectory manager that includes Z control via trapezoidal profile


---

## Phase 3: Data Logging & Visualization (Hours 16–24)

### Deliverables:

* [ ] Scripts to log and plot:
  * [ ] End-effector velocities
  * [ ] Joint velocities
* [ ] Visual confirmation of smooth motion (RViz or Gazebo screenshots)

### Tools:

* rqt\_plot, matplotlib, rosbag (optional)

---

## Phase 4: Bonus Addition – Compliance Control (Hours 24–36)

### Deliverables:

* [ ] Custom node for compliance control
* [ ] Demonstration of wall-following or constant force application

---

## Phase 5: Code Cleanup & Documentation (Hours 36–46)

### Deliverables:

* [ ] Clean and structured workspace (scripts, launch, config)
* [ ] README explaining:

  * [ ] Setup instructions
  * [ ] Control strategy
  * [ ] Assumptions & known limitations
* [ ] Optional: demo video/gif

---

## Phase 6: Final Testing & Submission (Hours 46–48)

### Deliverables:

* [ ] End-to-end test (spawn robot, lift, move, log, plot)
* [ ] Final archive or GitHub repo ready
* [ ] Checklist reviewed

---

## Total Estimated Time: 48 Hours

| Phase     | Task                        | Time Estimate |
| --------- | --------------------------- | ------------- |
| Env Setup | Docker + URDF + Testing     | 2 hrs         |
| Phase 1   | Base motion (XY at fixed Z) | 6 hrs         |
| Phase 2   | Lift control (Z motion)     | 8 hrs         |
| Phase 3   | Plotting & verification     | 8 hrs         |
| Phase 4   | Compliance control (bonus)  | 12 hrs        |
| Phase 5   | Cleanup + docs              | 10 hrs        |
| Phase 6   | Testing + deliverables      | 2 hrs         |

---
