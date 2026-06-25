#!/bin/bash
# ============================================================
#  Gesture Arm — Hand Tracker launcher
#  Run this AFTER run_simulation.sh is already running
# ============================================================

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

echo -e "${CYAN}[TRACKER] Starting hand gesture tracker...${NC}"

# Match the same domain ID as the simulation
export ROS_DOMAIN_ID=42

# ── Find the virtual environment ────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_PATHS=(
    "$HOME/ros2_ws/ros2_venv/bin/activate"
    "$HOME/ros2_venv/bin/activate"
    "/opt/ros2_venv/bin/activate"
)

VENV_FOUND=""
for venv in "${VENV_PATHS[@]}"; do
    if [ -f "$venv" ]; then
        VENV_FOUND="$venv"
        break
    fi
done

if [ -z "$VENV_FOUND" ]; then
    echo -e "${YELLOW}[WARN] No virtual environment found, trying system Python...${NC}"
    python3 "$SCRIPT_DIR/src/gesture_arm/gesture_arm/gesture_tracker.py"
else
    echo -e "${GREEN}[OK] Activating virtual environment: $VENV_FOUND${NC}"
    source "$VENV_FOUND"
    python3 "$SCRIPT_DIR/src/gesture_arm/gesture_arm/gesture_tracker.py"
fi
