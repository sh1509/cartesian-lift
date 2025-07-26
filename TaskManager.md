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

## Progress Log

| Task                          | Status           | Timestamp           |  Notes |
|-------------------------------|------------------|---------------------|--------|
| Start time                    | Started          | 25/07 01:00 - 01:30 |        |
| Resumed work                  | -----------      | 25/07 09:30         |        |
| URDF preparation              | ✅ Completed     | 25/07 12:18         |Custom URDF|
| Loaded with prismatic link    | ✅ Completed     | 25/07 16:05         |Time occurred in order to undertand the xacros for UR10E|


## EXTRA TODO
- ~~ADD Universal_Robots_ROS2_Description as submodules~~
- ~~Add the steps for running the launch files related to Gazebo~~

---

## Phase 1: Robot Model & Simulation (Hours 2–8)

  ### Deliverables:

  * [ ] Modified URDF: 6-DOF robot + vertical prismatic joint
  * [ ] Simulation world with robot spawned in Ignition Gazebo
* [ ] Basic Cartesian motion at fixed Z using trapezoidal profile

## Progress Log

| Task                          | Status          | Timestamp            | Notes |
|-------------------------------|------------------|---------------------|-------|
| Start time                    | Started          | 25/07 17:00         |       |
| Modified URDF                 | ✅ Completed     | 25/07 17:20         |Gazebo in docker failed to launch|
| Spawned Model                 | ✅ Completed     | 25/07 17:20         |       |
| Hardware Transmission         | Paused           | 25/07 19:10         |  gazebo_ros2_control plugin fails |
| Hardware Transmission         | Resumed          | 25/07 19:35         |  gazebo_ros2_control plugin fails |
| Fixed Transmission            | ✅ Completed     | 25/07 23:35         |  issues with xacros transmission and controllers.yaml |


## EXTRA TODO
- ~~create initial_positions.yaml for all the joints of the robot~~
-  ~~Add `apt install ros-humble-ros2-control ros-humble-ros2-controllers` in dependencies and installation~~
-  launch rviz with Gazebo (LATER)
-  ~~Add GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:/opt/ros/humble/share/gazebo_ros2_control in docker compose~~
-  Add all the modified xacros of ur_description (LATER)

---

## Phase 2: Z-Lift Control (Hours 8–16)

### Deliverables:

* [ ] Prismatic joint integrated into motion planning
* [ ] Cartesian motion on 2D plane (XY) at different Z heights
* [ ] Trajectory manager that includes Z control via trapezoidal profile

## Progress Log

| Task                          | Status          | Timestamp            | Notes |
|-------------------------------|------------------|---------------------|-------|
| Start time                    | Started          | 25/07 23:55         |       |
| Create control_pkg for motion | In-progress      | 26/07               | Use IKSolver for trajectory generation with added lift|
| trac_ik                       | Aborted          | 26/07  02:35        | No native support for ROS2, tried compiling by source but failed, plus no gurantee that the code will work with added lift|
| IK with lift                  | Started          | 26/07  10:16        |       |
| IK with lift                  | ✅ Completed          | 26/07  13:25        | created a module for generating IK solution with extra added lift, added a ros2 node for running cartesian motion without constant velocity      |
| Implement constant velocity motion| Started          | 26/07  13:25        |       |
| Implement constant velocity motion| ✅ Completed          | 26/07  17:20        |       |


## Extra TODO
- ~~Add ikpy, numpy, scipy in Dockerfile: pip install ikpy urdf-parser-py numpy~~
- ~~Add ur10e_without_lift.urdf to git~~


---

## Phase 3: Data Logging & Visualization (Hours 16–24)

### Deliverables:

* [ ] Scripts to log and plot:
  * [ ] End-effector velocities
  * [ ] Joint velocities
* [ ] Visual confirmation of smooth motion (RViz or Gazebo screenshots)

## Progress Log

| Task                          | Status          | Timestamp            | Notes |
|-------------------------------|------------------|---------------------|-------|
| Plot the end-effector velocities| ✅ Completed          | 26/07  18:45        |       |

### Tools:

* rqt\_plot, matplotlib, rosbag (optional)

---

## Phase 4: Bonus Addition – Compliance Control (Hours 24–36)
__Note: This is an optional phase that can be skipped if time does not permit.__
### Deliverables:

* [ ] Custom node for compliance control
* [ ] Demonstration of wall-following or constant force application

---

## Phase 5: Code Cleanup & Documentation (Hours 44–46)

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
| Phase 5   | Cleanup + docs              | 2 hrs         |
| Phase 6   | Testing + deliverables      | 2 hrs         |

---
