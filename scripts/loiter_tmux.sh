#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME=${SESSION_NAME:-loiter}
WS_DIR=${WS_DIR:-/home/nonsaya-n/repo/loiter/ros2_ws}
GLIM_CONFIG_PATH=${GLIM_CONFIG_PATH:-/home/nonsaya-n/glim_config}
FCU_URL=${FCU_URL:-serial:///dev/ttyTHS1:921600}
GLIM_NAMESPACE=${GLIM_NAMESPACE:-/glim_ros}
TARGET_TOPIC=${TARGET_TOPIC:-/mavros/odometry/out}

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux が見つかりません。 sudo apt-get install -y tmux でインストールしてください。" >&2
  exit 1
fi

tmux has-session -t "$SESSION_NAME" 2>/dev/null && {
  echo "既存のセッション: $SESSION_NAME が存在します。 tmux attach -t $SESSION_NAME で接続してください。" >&2
  exit 0
}

livox_cmd="source /opt/ros/humble/setup.bash; source $WS_DIR/install/setup.bash; ros2 launch livox_mid360_bringup livox_mid360.launch.py"

# GLIMはソースWSがあれば読み込む
glim_source="[ -f \"$HOME/glim_src_ws/install/setup.bash\" ] && source \"$HOME/glim_src_ws/install/setup.bash\" || true"
glim_cmd="source /opt/ros/humble/setup.bash; $glim_source; ros2 run glim_ros glim_rosnode --ros-args -p config_path:=$GLIM_CONFIG_PATH"

wait_cmd="source /opt/ros/humble/setup.bash; source $WS_DIR/install/setup.bash; ros2 run glim_bringup wait_for_topic.py --topic $GLIM_NAMESPACE/odom --timeout 180 --min_publishers 1"

mavros_cmd="$wait_cmd && ros2 launch mavros apm.launch fcu_url:=$FCU_URL"

bridge_cmd="$wait_cmd && ros2 launch glim_mavros_bridge odom_bridge.launch.py \\
  glim_namespace:=$GLIM_NAMESPACE \\
  use_corrected:=false \\
  publish_rate_hz:=15.0 \\
  odom_child_frame_id:=base_link \\
  restamp_source:=now \\
  reject_older_than_ms:=200.0 \\
  publish_immediately:=true \\
  target_topic:=$TARGET_TOPIC"

# セッション開始
tmux new-session -d -s "$SESSION_NAME" "bash -lc '$livox_cmd'"

# Pane 2: GLIM
tmux split-window -h -t "$SESSION_NAME":0 "bash -lc '$glim_cmd'"

# Pane 3: MAVROS（GLIM待ち→起動）
tmux split-window -v -t "$SESSION_NAME":0.0 "bash -lc '$mavros_cmd'"

# Pane 4: Bridge（GLIM待ち→起動）
tmux split-window -v -t "$SESSION_NAME":0.1 "bash -lc '$bridge_cmd'"

tmux select-layout -t "$SESSION_NAME":0 tiled

echo "起動中のtmuxセッション: $SESSION_NAME"
echo "接続: tmux attach -t $SESSION_NAME"




