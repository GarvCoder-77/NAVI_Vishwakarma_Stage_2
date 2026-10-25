#!/usr/bin/env bash
# ==============================================================================
# BLIND ASSISTANT ROBOT — SINGLE-COMMAND MASTER COMPETITION LAUNCHER
# ==============================================================================
# Usage:
#   ./start_demo.sh                (Runs autonomous exploration with 120s time cap)
#   ./start_demo.sh 90             (Runs autonomous exploration with 90s time cap)
#   ./start_demo.sh 180            (Runs autonomous exploration with 180s time cap)
# ==============================================================================

set -e

# Parse optional maximum exploration time argument (default: 120 seconds)
MAX_TIME="${1:-120.0}"

# Terminal color codes
C_CYAN='\033[0;36m'
C_GREEN='\033[0;32m'
C_YELLOW='\033[1;33m'
C_RED='\033[0;31m'
C_BOLD='\033[1m'
C_RESET='\033[0m'

clear
cat << "EOF"
  ____  _ _           _     _                         _             _   
 | __ )| (_)_ __   __| |   / \   ___ ___ _   _  __ _| |_ ___  _ __| |_ 
 |  _ \| | | '_ \ / _` |  / _ \ / __/ __| | | |/ _` | __/ _ \| '__| __|
 | |_) | | | | | | (_| | / ___ \\__ \__ \ |_| | (_| | || (_) | |  | |_ 
 |____/|_|_|_| |_|\__,_|/_/   \_\___/___/\__,_|\__,_|\__\___/|_|   \__|
              COMPETITION INDOOR AUTONOMOUS DEMONSTRATOR
EOF

echo -e "${C_CYAN}===============================================================================${C_RESET}"
echo -e "${C_BOLD}Hardware:${C_RESET} Arduino UNO Q (Dual Brain) + YDLidar X4 + USB Monocular Webcam"
echo -e "${C_BOLD}Perception:${C_RESET} Monocular Webcam + 2D LiDAR Ray-Casting Fusion (Wall Rejection Active)"
echo -e "${C_BOLD}Domain ID:${C_RESET} ${C_GREEN}42${C_RESET} (Isolated Wi-Fi Mesh) | ${C_BOLD}Max Exploration Cap:${C_RESET} ${C_YELLOW}${MAX_TIME}s${C_RESET}"
echo -e "${C_CYAN}===============================================================================${C_RESET}"

# 1. Source ROS 2 Base Distribution
echo -e "\n${C_BOLD}[1/4] Checking ROS 2 Environment...${C_RESET}"
ROS_DISTRO_PATH=""
for distro in jazzy humble iron rolling foxy; do
    if [ -f "/opt/ros/${distro}/setup.bash" ]; then
        ROS_DISTRO_PATH="/opt/ros/${distro}/setup.bash"
        break
    fi
done

if [ -z "$ROS_DISTRO_PATH" ]; then
    echo -e "${C_RED}✗ Error: No standard ROS 2 installation found in /opt/ros!${C_RESET}"
    exit 1
fi

source "$ROS_DISTRO_PATH"
echo -e "  ${C_GREEN}✓ Sourced ROS 2 from ${ROS_DISTRO_PATH}${C_RESET}"

# 2. Source Robot Workspace
echo -e "\n${C_BOLD}[2/4] Sourcing Robot Workspace...${C_RESET}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_FOUND=false

CANDIDATE_WS=(
    "${SCRIPT_DIR}/ros2_simulation_ws"
    "${HOME}/.gemini/antigravity/scratch/ros2_simulation_ws"
    "${SCRIPT_DIR}"
)

for ws in "${CANDIDATE_WS[@]}"; do
    if [ -f "${ws}/install/setup.bash" ]; then
        source "${ws}/install/setup.bash"
        echo -e "  ${C_GREEN}✓ Sourced workspace: ${ws}/install/setup.bash${C_RESET}"
        WS_FOUND=true
        break
    fi
done

if [ "$WS_FOUND" = false ]; then
    echo -e "  ${C_YELLOW}! Warning: Pre-built install/setup.bash not found. Attempting local package lookup.${C_RESET}"
fi

# Export Domain ID and CycloneDDS for clean Wi-Fi discovery
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1

# 3. Network & Uno Q Pairing Check
echo -e "\n${C_BOLD}[3/4] Probing Robot Base Connection on ROS_DOMAIN_ID=42...${C_RESET}"
echo -e "  Listening for active robot hardware topics (/scan, /odom, /camera/image_raw)..."

# 4-second quick topic probe
DISCOVERED_TOPICS=$(ros2 topic list 2>/dev/null || true)

if echo "$DISCOVERED_TOPICS" | grep -q "/scan"; then
    echo -e "  ${C_GREEN}✓ Connected to Arduino Uno Q! YDLidar (/scan) online.${C_RESET}"
else
    echo -e "  ${C_YELLOW}! Notice: Robot base not broadcasting yet.${C_RESET}"
    echo -e "    Make sure ${C_BOLD}blind_assistant_robot_app${C_RESET} is deployed and running in Arduino App Lab on Uno Q."
    echo -e "    Proceeding — SLAM and visualizer will link automatically as soon as Uno Q joins Wi-Fi."
fi

# Safety trap to halt motors if script is killed
cleanup() {
    echo -e "\n${C_RED}[SAFETY] Halting robot motors and stopping processes...${C_RESET}"
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
    echo -e "${C_GREEN}✓ Standstill confirmed. Safe to power down.${C_RESET}"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 4. Launch Master Competition Stack
echo -e "\n${C_BOLD}[4/4] Starting Competition Master Stack...${C_RESET}"
echo -e "  - ${C_CYAN}SLAM Toolbox:${C_RESET} Real-Time 2D Occupancy Grid Mapping"
echo -e "  - ${C_CYAN}Perception:${C_RESET} Monocular Webcam + 2D LiDAR Ray-Casting Fusion (Wall Rejection)"
echo -e "  - ${C_CYAN}Exploration:${C_RESET} Autonomous Frontier Engine (Time Cap: ${MAX_TIME}s)"
echo -e "  - ${C_CYAN}Visualizer:${C_RESET} Native Real-Time SLAM Map Window with Discovered Pins"
echo -e "  - ${C_CYAN}Voice Guidance:${C_RESET} Assistive Audio Feedback"
echo -e "\n${C_GREEN}>>> Set the robot on the room floor, step back, and press [ENTER] to begin! <<<${C_RESET}"
read -r

# Execute master launch file
ros2 launch blind_assistant_robot competition_demo.launch.py \
    use_sim_time:=false \
    max_exploration_time:="${MAX_TIME}"
