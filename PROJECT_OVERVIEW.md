# Project Overview
## Gesture-Controlled Robotic Arm Simulation
**Aaditya Sabharwal | Thapar Institute of Engineering & Technology | RAI Workshop 2025**

---

## What is this project?

This project simulates a **UR5e industrial robotic arm** that can be controlled using hand gestures captured from a standard webcam. It runs on **ROS 2** (Robot Operating System 2) and visualises everything in real-time using **RViz2**.

The system has three modes:
- **TELEOP** — Your hand movements directly control the robot arm in real-time
- **AUTO** — The robot works fully on its own, picking up coloured cubes and placing them in a box
- **STOP** — Emergency halt; freezes the robot at its exact current position

---

## How does it work?

### 1. Hand Tracking (gesture_tracker.py)
A webcam feed is processed using two MediaPipe models simultaneously:

- **MediaPipe Pose** — Detects body landmarks (shoulders, wrists, elbows) to determine which mode the robot should be in. Raising your right hand above your shoulder activates TELEOP; raising your left hand triggers STOP.
- **MediaPipe Hands** — Detects **21 hand landmarks** (fingertips, knuckles, wrist) for precise control. Your hand's 3D position in space is extracted as a spatial vector (Right/Up/Forward relative to your shoulder).

**Gripper control** uses a scale-invariant fist detection algorithm — the ratio of thumb-to-index distance vs wrist-to-knuckle distance. This works reliably at any camera distance.

The tracking data is sent to the robot controller over a **UDP socket** at 30 fps.

### 2. Robot Controller (gesture_publisher.py)
The brain of the system. It receives the hand position data and processes it through a multi-stage control pipeline:

1. **1€ Adaptive Filter** — Smooths hand position data. When your hand moves slowly, the filter is aggressive (kills jitter). When you move fast, it lets the signal through immediately (stays responsive).
2. **Velocity Dead Zone** — If hand movement between frames is below 3mm, the arm target freezes completely. This eliminates micro-jitter when holding still.
3. **Iterative Inverse Kinematics** — 8 refinement passes compute the 6 joint angles needed to reach the target position. Each pass corrects the error from the previous one.
4. **Joint-Level EMA** — After IK, a secondary exponential moving average smooths out any discontinuities in the joint angle solution.
5. **Joint Interpolator** — Limits how fast each joint can move (4.5 rad/s max), creating smooth, continuous motion.

> **What is Inverse Kinematics?**
> Given a desired position in 3D space (e.g. "move the gripper to X=-0.5m, Y=-0.2m, Z=0.05m"), IK works backwards to calculate exactly how much each joint needs to rotate. It uses the Law of Cosines (triangle geometry) to solve for the elbow angle, then trigonometry for the shoulder and base.

### 3. Visualisation (RViz2)
RViz2 is ROS 2's 3D visualiser. It reads the joint angles and renders the robot arm, cubes, drop box, and scene in real-time. The scene includes:
- **Dark metallic table** with edge highlighting
- **Subtle grid floor** for spatial reference
- **Teal drop box** with walls
- **Two-finger gripper** that visually opens and closes
- **Proximity glow rings** around cubes when the gripper approaches
- **Target ghost sphere** showing where the arm is trying to reach
- **Real-time HUD** displaying mode, gripper state, and score

---

## Mode Switching — Hysteresis Lock

A critical feature for usability. The mode system uses **hysteresis** — different thresholds for entering vs exiting a mode:

- **Entering TELEOP**: Right wrist must be clearly above shoulder (easy to enter)
- **Staying in TELEOP**: Once locked, your wrist can move freely without accidentally switching modes — even if your elbow drops below your shoulder
- **Exiting TELEOP**: Right wrist must drop well below shoulder for 15+ consecutive frames (deliberate exit only)
- **STOP**: Left wrist above shoulder triggers E-STOP within 8 frames (fast, safety-critical)

This prevents the most common usability problem — accidentally switching from TELEOP to AUTO when lowering your arm to control the robot downward.

---

## TELEOP Features

| Feature | Description |
|---|---|
| **Delta-based control** | Hand movement relative to a calibrated reference point maps to robot movement |
| **Periodic re-calibration** | Reference point gently drifts every 5 seconds to eliminate accumulated offset |
| **Snap assist** | When gripping near a cube, the arm is magnetically pulled toward the cube centre (20% bias) |
| **Proximity glow** | Yellow ring appears when approaching a cube; turns green with **[GRAB!]** when in pick range |
| **True E-STOP** | STOP freezes the arm at its exact position — no drifting to a home position |

---

## AUTO Mode — How does the robot work on its own?

AUTO mode uses a **Finite State Machine** — the robot is always in exactly one state and knows what to do next:

```
SCAN → APPROACH → DESCEND → GRAB → LIFT → CARRY → DROP → HOME → repeat
```

1. **SCAN** — Finds the nearest unplaced cube
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
| **MediaPipe (Google)** | Real-time hand + pose detection |
| **UR5e URDF** | Official Universal Robots 3D model |
| **RViz2** | 3D visualisation |
| **UDP Socket** | Low-latency data transfer (tracker → controller) |
| **Iterative IK** | 8-pass geometric joint angle solver with FK correction |
| **1€ Filter** | Adaptive smoothing (Géry Casiez et al.) |

---

## System Architecture

```
[Webcam]
   ↓
[gesture_tracker.py]         — MediaPipe Pose + Hands detection
   ↓ UDP (port 9870)
[gesture_publisher.py]       — 1€ filter → velocity dead zone → IK → joint EMA → interpolator
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
