MID360ドライバの起動
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash
ros2 launch livox_mid360_bringup livox_mid360.launch.py

GLIMの起動
source /opt/ros/humble/setup.bash
source ~/glim_src_ws/install/setup.bash
ros2 run glim_ros glim_rosnode --ros-args -p config_path:=/home/nonsaya-n/glim_config


ビューアを無効にする方法
libstandard_viewer.soを削除しますconfig_ros.json。すると、GLIM は GUI なしで起動します。

MAVROSの起動
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyTHS1:921600

ブリッジ起動
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash
ros2 launch glim_mavros_bridge odom_bridge.launch.py \
  glim_namespace:=/glim_ros \
  use_corrected:=false \
  publish_rate_hz:=15.0 \
  odom_child_frame_id:=base_link \
  restamp_source:=now \
  reject_older_than_ms:=200.0 \
  publish_immediately:=true \
  target_topic:=/mavros/odometry/out


一括起動（単一コマンド）
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash
ros2 launch glim_bringup loiter.launch.py \
  fcu_url:=serial:///dev/ttyTHS1:921600 \
  glim_config_path:=/home/nonsaya-n/glim_config \
  use_corrected:=false \
  publish_rate_hz:=15.0 \
  odom_child_frame_id:=base_link \
  restamp_source:=now \
  reject_older_than_ms:=200.0 \
  publish_immediately:=true \
  target_topic:=/mavros/odometry/out


二段起動（推奨: GLIM起動待ちしやすい）
# 1) MID360 + GLIM
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash
ros2 launch glim_bringup sensing.launch.py \
  glim_config_path:=/home/nonsaya-n/glim_config

# 2) MAVROS + Bridge（GLIMが安定してから）
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash
ros2 launch glim_bringup flight.launch.py \
  fcu_url:=serial:///dev/ttyTHS1:921600 \
  glim_namespace:=/glim_ros \
  use_corrected:=false \
  publish_rate_hz:=15.0 \
  odom_child_frame_id:=base_link \
  restamp_source:=now \
  reject_older_than_ms:=200.0 \
  publish_immediately:=true \
  target_topic:=/mavros/odometry/out


一発起動（GLIM準備完了待ち）
source /opt/ros/humble/setup.bash
source /home/nonsaya-n/repo/loiter/ros2_ws/install/setup.bash
ros2 launch glim_bringup loiter_ready.launch.py \
  fcu_url:=serial:///dev/ttyTHS1:921600 \
  glim_config_path:=/home/nonsaya-n/glim_config \
  glim_namespace:=/glim_ros \
  use_corrected:=false \
  publish_rate_hz:=15.0 \
  odom_child_frame_id:=base_link \
  restamp_source:=now \
  reject_older_than_ms:=200.0 \
  publish_immediately:=true \
  target_topic:=/mavros/odometry/out


tmux一発起動（各コンポーネントを別ペインで起動・GLIM待ち）
sudo apt-get install -y tmux  # 未インストールなら
/home/nonsaya-n/repo/loiter/scripts/loiter_tmux.sh
  # 終了時: 各ペインで Ctrl-C、または別端末から tmux kill-session -t loiter






  
