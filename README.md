# Robotic Arm Kinematics & Path Planning 🤖🤖

![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Language-Python3-green)

**Developed by:** Ata Ul Musawir  
**Program:** BS Electrical Engineering - GIK Institute (GIKI)

## 📌 Project Overview
This repository contains the full production solution for a 6-DOF manipulator arm kinematics project. The primary engineering goal is to calculate Inverse Kinematics (IK) for a custom robotic arm and execute smooth, collision-free trajectory splines moving from Point A to Point B without physical workspace collisions.

## 🏗️ Architecture & Signal Pipeline
1. **Input Stage:** Target End-Effector Cartesian Coordinates (X, Y, Z).
2. **Process Stage:** Analytical Inverse Kinematics + TRAC-IK solver & 5th-order quintic spline generation.
3. **Output Stage:** Asynchronous `FollowJointTrajectory` action server dispatching trajectory points to physics simulation.

## 📊 Results & Output
- **Target Point A:** `(0.35, -0.15, 0.45)` -> IK Angles Computed Successfully.
- **Target Point B:** `(0.45, 0.20, 0.60)` -> IK Angles Computed Successfully.
- Smooth velocity and acceleration curves generated and verified.
