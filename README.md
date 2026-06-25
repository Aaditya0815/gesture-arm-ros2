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

- **TELEOP Mode** → Your hand controls the robot arm live via webcam
- **AUTO Mode** → The arm autonomously picks up coloured cubes and sorts them into a box using Inverse Kinematics

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

## Project Structure

```
gesture_arm_submission/
├── run_simulation.sh          ← One-click launcher (START HERE)
├── run_tracker.sh             ← Hand tracker launcher
├── README.md                  ← This file
├── PRESENTATION_NOTES.md      ← Full technical documentation
├── ros2_ws/
│   └── src/
│       └── gesture_arm/
│           ├── gesture_arm/
│           │   ├── gesture_publisher.py   ← Robot brain (IK, AUTO mode, scene)
│           │   └── gesture_tracker.py     ← Hand CV tracking (MediaPipe)
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
| **Inverse Kinematics** | Calculates joint angles from desired 3D position |
| **MediaPipe Hands** | Detects 21 hand landmarks from webcam in real-time |
| **UDP Socket** | Low-latency channel from tracker to robot controller |
| **State Machine** | AUTO mode logic: SCAN→PICK→CARRY→DROP→RESET |
| **Cosine Interpolation** | Smooth S-curve motion between joint positions |
| **TF / robot_state_publisher** | Broadcasts live positions of all robot links |
| **RViz2 Markers** | Renders cubes, box, table, and gripper ball |

---

*Built with ROS 2 | UR5e | MediaPipe | Python*
