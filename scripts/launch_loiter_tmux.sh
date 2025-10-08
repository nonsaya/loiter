#!/usr/bin/env bash
set -euo pipefail

SESSION="loiter"
WS_DIR="/home/nonsaya-n/repo/loiter/ros2_ws"
GLIM_CONFIG="/home/nonsaya-n/repo/loiter/ros2_ws/src/glim_bringup/config/glim_config"
SERIAL_DEV="/dev/ttyTHS1:921600"

# install tmux if missing
if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux not found. Installing..."
  sudo apt update && sudo apt install -y tmux
fi

# ensure ROS env is sourced command-safe
ROS_SETUP() {
  echo "source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"
}

# kill existing session
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
fi

# new session
TMUX_CMD=(tmux new-session -d -s "$SESSION")
"${TMUX_CMD[@]}" bash -lc "$(ROS_SETUP); ros2 launch livox_mid360_bringup livox_mid360.launch.py"

# split for GLIM
TMUX_SPLIT_GLIM=(tmux split-window -h -t "$SESSION":0)
"${TMUX_SPLIT_GLIM[@]}" bash -lc "$(ROS_SETUP); /opt/ros/humble/lib/glim_ros/glim_rosnode --ros-args -p config_path:=${GLIM_CONFIG}"

# split for MAVROS (below left)
tmux split-window -v -t "$SESSION":0.0 bash -lc "$(ROS_SETUP); ros2 launch mavros apm.launch fcu_url:=serial://${SERIAL_DEV} tgt_system:=1"

# split for Bridge (below right)
tmux split-window -v -t "$SESSION":0.1 bash -lc "$(ROS_SETUP); ros2 launch glim_mavros_bridge bridge.launch.py mirror_to_odometry_out:=false publish_vision_topics:=false override_stamp_with_now:=false frame_id:=odom child_frame_id:=base_link"

# layout and attach
tmux select-layout -t "$SESSION" tiled

echo "Attach with: tmux attach -t ${SESSION}"
tmux attach -t "$SESSION"
