# 🤖 Gesture-Controlled Robotic Arm Simulation
### Aaditya Sabharwal | Thapar University (TIET) | RAI Workshop

---

## Development Environment

> ⚠️ This project was **developed and tested on the following setup**. Running on a different environment may require minor adjustments.

| Component | Version Used |
|---|---|
| **OS** | Ubuntu 26.04 LTS |
| **ROS 2 Distro** | ROS 2 Lyrical Luth |
| **Python** | 3.12 |
| **MediaPipe** | 0.10.x |
| **Hardware** | Standard laptop webcam |

**Recommended:** Run on a machine with **ROS 2 Lyrical** for guaranteed compatibility.
Jazzy (2024) should also work. Humble (2022) may require package version adjustments.

---

## What This Project Does

A real-time **gesture-controlled UR5e robotic arm simulation** built on ROS 2.

- **TELEOP Mode** → Your hand controls the robot arm live via webcam — with hysteresis mode locking, velocity dead zone, snap-assist picking, and a two-finger gripper
- **AUTO Mode** → The arm autonomously picks up coloured cubes and sorts them into a box using Inverse Kinematics and a finite state machine
- **STOP Mode** → Emergency halt; freezes the robot at its exact current position

---

## Prerequisites

Make sure the following are installed on the machine:
```bash
# ROS 2 Lyrical (recommended) or Jazzy
# ur_description package:
sudo apt install ros-$(. /opt/ros/*/setup.bash && echo $ROS_DISTRO)-ur-description

# Python dependencies for hand tracker:
pip install mediapipe opencv-python numpy
```

---

## How to Run (2 steps)

### Step 1 — Launch the simulation
```bash
cd ~/gesture_arm_submission
bash run_simulation.sh
```
> RViz2 will open. The arm starts in **AUTO mode** immediately.

### Step 2 — (Optional) Launch the hand tracker
Open a **second terminal**:
```bash
cd ~/gesture_arm_submission
bash run_tracker.sh
```
> A webcam window opens. Your hand now controls the arm in real-time.

---

## ⚠️ Important: If you are in a lab with other ROS machines

The scripts automatically set `ROS_DOMAIN_ID=42` — a private channel that isolates your simulation from any other robots running in the room. You do **not** need to do anything extra.

If the robot model appears as **"Status: Error"** in RViz:
```bash
# Close everything, then open a fresh terminal and run:
export ROS_DOMAIN_ID=42
bash run_simulation.sh
```

---

## Manual Run (without scripts)

```bash
# Terminal 1
export ROS_DOMAIN_ID=42
source /opt/ros/<your-distro>/setup.bash
cd ~/gesture_arm_submission/ros2_ws
colcon build --packages-select gesture_arm
source install/setup.bash
ros2 launch gesture_arm demo.launch.py

# Terminal 2 (hand tracker)
export ROS_DOMAIN_ID=42
source ~/ros2_ws/ros2_venv/bin/activate  # or your venv path
python3 src/gesture_arm/gesture_arm/gesture_tracker.py
```

---

## Control Guide

| Gesture | Action |
|---|---|
| **Raise right hand** above shoulder | Enter **TELEOP** mode (locks in) |
| **Move hand** in 3D space | Control robot arm position |
| **Make a fist** | Close gripper (pick up cube) |
| **Open hand** | Open gripper (release cube) |
| **Raise left hand** above shoulder | **Emergency STOP** — arm freezes instantly |
| **Drop right hand** well below shoulder | Exit TELEOP → return to **AUTO** |

---

## Project Structure

```
gesture-arm-ros2/
├── run_simulation.sh          ← One-click launcher (START HERE)
├── run_tracker.sh             ← Hand tracker launcher
├── README.md                  ← This file
├── PROJECT_OVERVIEW.md        ← Technical deep-dive
├── ros2_ws/
│   └── src/
│       └── gesture_arm/
│           ├── gesture_arm/
│           │   ├── gesture_publisher.py   ← Robot brain (IK, scene, control)
│           │   └── gesture_tracker.py     ← Hand tracking (MediaPipe + Pose)
│           ├── launch/
│           │   └── demo.launch.py         ← ROS 2 launch file
│           ├── urdf/
│           │   └── simple_arm.urdf        ← Fallback URDF
│           ├── package.xml
│           └── setup.py
```

---

## Key Technical Concepts

| Concept | What it does in this project |
|---|---|
| **ROS 2** | Middleware connecting tracker → robot brain → RViz |
| **Inverse Kinematics** | Iterative solver (8 refinement passes) calculates joint angles from desired 3D position |
| **MediaPipe Pose + Hands** | Detects body pose (mode switching) and 21 hand landmarks (control + gripper) |
| **1€ Filter** | Adaptive low-pass filter — smooth when still, responsive when moving fast |
| **Velocity Dead Zone** | Freezes arm completely when hand movement is below threshold — eliminates micro-jitter |
| **Joint EMA** | Post-IK exponential moving average smooths out IK discontinuities |
| **Hysteresis Mode Lock** | Once in TELEOP, requires deliberate exit gesture to prevent accidental mode switches |
| **Snap Assist** | Magnetic pull toward nearest cube when gripping — makes picking intuitive |
| **UDP Socket** | Low-latency channel from tracker to robot controller |
| **State Machine** | AUTO mode logic: SCAN→PICK→CARRY→DROP→HOME→repeat |
| **Cosine Interpolation** | Smooth S-curve motion between joint positions |
| **TF / robot_state_publisher** | Broadcasts live positions of all robot links |

---

*Built with ROS 2 | UR5e | MediaPipe | Python*
