# Project Overview
## Gesture-Controlled Robotic Arm Simulation
**Aaditya Sabharwal | Thapar Institute of Engineering & Technology | RAI Workshop 2025**

---

## What is this project?

This project simulates a **UR5e industrial robotic arm** that can be controlled using hand gestures captured from a standard webcam. It runs on **ROS 2** (Robot Operating System 2) and visualises everything in real-time using **RViz2**.

The system has three modes:
- **TELEOP** — Your hand movements directly control the robot arm in real-time
- **AUTO** — The robot works fully on its own, picking up coloured cubes and placing them in a box
- **STOP** — Emergency halt; freezes the robot immediately in its current position

---

## How does it work?

### 1. Hand Tracking (gesture_tracker.py)
A webcam feed is processed by **Google's MediaPipe** library, which detects **21 landmarks on your hand** (fingertips, knuckles, wrist) at 30 frames per second. The position of your **index fingertip** maps to where the robot moves. The **distance between your thumb and index finger** controls the gripper — pinch to close, open hand to release. This data is sent to the robot controller over a **UDP socket**.

### 2. Robot Controller (gesture_publisher.py)
The brain of the system. It receives the hand position data and uses **Inverse Kinematics (IK)** to calculate the 6 joint angles needed to move the robot arm to that position. It publishes these angles to ROS 2, which moves the simulated arm.

> **What is Inverse Kinematics?**
> Given a desired position in 3D space (e.g. "move the gripper to X=-0.5m, Y=-0.2m, Z=0.05m"), IK works backwards to calculate exactly how much each joint needs to rotate. It uses the Law of Cosines (triangle geometry) to solve for the elbow angle, then trigonometry for the shoulder and base.

### 3. Visualisation (RViz2)
RViz2 is ROS 2's 3D visualiser. It reads the joint angles and renders the robot arm, cubes, drop box, and scene in real-time. The coloured cubes, table, and box are custom 3D markers published by our code.

---

## AUTO Mode — How does the robot work on its own?

AUTO mode uses a **Finite State Machine** — the robot is always in exactly one state and knows what to do next:

```
SCAN → APPROACH → DESCEND → GRAB → LIFT → CARRY → DROP → HOME → repeat
```

1. **SCAN** — Finds the nearest cube using Euclidean distance
2. **APPROACH** — Moves above the cube using IK
3. **DESCEND** — Lowers to cube height
4. **GRAB** — Closes gripper, attaches cube
5. **LIFT** — Rises back up
6. **CARRY** — Moves to the drop box
7. **DROP** — Lowers into box, releases cube
8. **HOME** — Returns to rest position
9. Repeats until all cubes are placed, then resets with new random positions

Motion between positions uses **cosine interpolation** — smooth acceleration and deceleration like a real robot.

---

## Key Technologies

| Technology | Role |
|---|---|
| **ROS 2 (Lyrical Luth)** | Middleware connecting all components |
| **Python 3.12** | All logic and control code |
| **MediaPipe (Google)** | Real-time hand landmark detection |
| **UR5e URDF** | Official Universal Robots 3D model |
| **RViz2** | 3D visualisation |
| **UDP Socket** | Low-latency data transfer from tracker to controller |
| **Analytical IK** | Geometric joint angle solver (Law of Cosines) |

---

## System Architecture

```
[Webcam]
   ↓
[gesture_tracker.py]         — detects hand landmarks via MediaPipe
   ↓ UDP (port 5005)
[gesture_publisher.py]       — solves IK, runs state machine, publishes joints
   ↓ /joint_states topic
[robot_state_publisher]      — calculates 3D positions of all robot links
   ↓ /tf topic
[RViz2]                      — renders the arm and scene in 3D
```

---

## The Hardest Bug We Fixed

During development, the robot was correctly picking and placing cubes logically, but **visually in RViz2 the arm appeared to be operating in completely empty space** — 180° away from the actual objects.

The cause was a **hidden 180° rotation** built into the official UR5e URDF file by Universal Robots (a known coordinate convention fix). Our Inverse Kinematics math was not accounting for this rotation, so the mathematical model and the visual model were operating in opposite coordinate systems.

The fix was adding a single `π` radian rotation to our Forward Kinematics function, synchronising the two systems completely.

---

## Development Environment

| Component | Version |
|---|---|
| OS | Ubuntu 26.04 LTS |
| ROS 2 | Lyrical Luth |
| Python | 3.12 |

---

*Built with ROS 2 · Universal Robots UR5e · MediaPipe · Python*
