XP-Robotics-Assessment

Vision-Based Pick and Place using ROS 2 (Gazebo + RViz)

Overview

This repository contains a ROS 2–based robotic manipulation project demonstrating a vision-based pick-and-place system in simulation.
The project integrates robot simulation, perception, motion planning, and execution using Gazebo and RViz.

The implementation is structured to reflect real-world robotics system architecture and is suitable for technical assessments, hiring assignments, and academic evaluation.

Objective

The objective of this project is to design and demonstrate a complete vision-based pick-and-place pipeline where a robotic manipulator:

Detects objects using an eye-in-hand camera

Estimates object pose

Plans collision-free motion

Picks the object

Places it at a predefined target location

All components run entirely in simulation and are visualized in RViz.

System Architecture

The project follows a Task and Motion Planning (TAMP) inspired architecture:

Gazebo Simulation
 ├─ Robot Arm + Gripper
 ├─ Eye-in-Hand Camera
 ├─ Objects + Table
        ↓
ROS 2 Control (ros2_control)
        ↓
Perception Stack
 ├─ Camera Driver
 ├─ Object Detection
 ├─ Pose Estimation
        ↓
Motion Planning
 ├─ MoveIt 2
 ├─ IK and Collision Checking
        ↓
Execution Layer
 ├─ Trajectory Execution
 ├─ Gripper Control

Key Features

ROS 2 modular node-based architecture

Gazebo simulation with robotic arm and gripper

Eye-in-hand camera configuration

Object detection and pose estimation

TF2-based coordinate transformations

Motion planning using MoveIt 2

End-to-end pick-and-place execution

RViz visualization of robot, TF frames, and trajectories

Technologies Used

ROS 2

Gazebo

RViz

MoveIt 2

TF2

OpenCV / Vision techniques

C++ and Python

Repository Structure
XP-Robotics-Assessment/
├── docker/
├── mycobot_bringup/
├── mycobot_description/
├── mycobot_gazebo/
├── mycobot_interfaces/
├── mycobot_moveit_config/
├── mycobot_moveit_demos/
├── mycobot_mtc_demos/
├── mycobot_mtc_pick_place_demo/
├── mycobot_ros2/
├── mycobot_system_tests/
├── LICENSE
└── README.md

Getting Started
Prerequisites

Ubuntu 20.04 or 22.04

ROS 2 (Foxy or Humble)

Gazebo

colcon

MoveIt 2

Build Instructions
colcon build
source install/setup.bash

Running the Simulation

Launch the Gazebo simulation with the robot and environment:

ros2 launch mycobot_bringup mycobot_gazebo.launch.py


Use RViz to visualize:

Robot model

TF frames

Camera image stream

Planned trajectories

Detected object pose

Pick and Place Pipeline
Pick Phase

Move to pre-grasp pose

Open gripper

Approach the object

Close gripper

Lift the object

Place Phase

Move to pre-place pose

Lower the object

Open gripper

Retreat

Learning Outcomes

Understanding ROS 2 system architecture

Integrating perception with manipulation

Working with TF trees and coordinate frames

Motion planning using MoveIt 2

Designing end-to-end robotic manipulation pipelines

License

This project follows the original open-source license of the base repository used for learning and experimentation.

Author

Aayushi Kumari
B.Tech – Computer Science and Engineering
BIT Mesra
