#!/bin/bash
# ============================================================
#  Gesture Arm — One-click launcher
#  Run this script to build and launch the simulation
# ============================================================

set -e  # Stop on any error

# ── Colour output ──────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

echo -e "${BOLD}${CYAN}"
echo "  ╔══════════════════════════════════════════╗"
echo "  ║   Gesture-Controlled Robotic Arm Sim     ║"
echo "  ║   Aaditya Sabharwal | Thapar | TIET RAI  ║"
echo "  ╚══════════════════════════════════════════╝"
echo -e "${NC}"

# ── Step 1: Find ROS 2 installation ──────────────────────────
ROS_DISTRO_PATH=""
for path in /opt/ros/*/setup.bash; do
    if [ -f "$path" ]; then
        ROS_DISTRO_PATH="$path"
        break
    fi
done

if [ -z "$ROS_DISTRO_PATH" ]; then
    echo -e "${RED}[ERROR] ROS 2 not found in /opt/ros/. Is ROS 2 installed?${NC}"
    exit 1
fi
echo -e "${GREEN}[OK] Found ROS 2 at: $ROS_DISTRO_PATH${NC}"
source "$ROS_DISTRO_PATH"

# ── Step 2: Set unique domain ID (avoids conflicts in labs) ──
export ROS_DOMAIN_ID=42
echo -e "${GREEN}[OK] ROS_DOMAIN_ID set to 42 (private channel)${NC}"

# ── Step 3: Find this script's directory (the workspace root) ─
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/ros2_ws"

# ── Step 4: Build the workspace ───────────────────────────────
echo -e "\n${YELLOW}[BUILD] Building gesture_arm package...${NC}"
cd "$WS_DIR"
colcon build --packages-select gesture_arm --symlink-install 2>&1 | tail -5

if [ $? -ne 0 ]; then
    echo -e "${RED}[ERROR] Build failed. Check the output above.${NC}"
    exit 1
fi
echo -e "${GREEN}[OK] Build successful!${NC}"

# ── Step 5: Source the workspace ─────────────────────────────
source "$WS_DIR/install/setup.bash"
echo -e "${GREEN}[OK] Workspace sourced${NC}"

# ── Step 6: Launch ───────────────────────────────────────────
echo -e "\n${CYAN}[LAUNCH] Starting simulation...${NC}"
echo -e "${YELLOW}  → RViz2 will open with the robot arm and scene"
echo -e "  → Arm starts in AUTO mode — picks and places cubes automatically"
echo -e "  → To use hand control: run gesture_tracker.py in a second terminal${NC}\n"

ros2 launch gesture_arm demo.launch.py
