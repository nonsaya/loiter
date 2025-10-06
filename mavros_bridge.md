# GLIM ↔ MAVROS ブリッジ手順（外部航法 / Vision・Odometry 経路）

このドキュメントだけで、GLIMの推定結果をMAVROS経由でArduPilot EKFへ供給するブリッジを構築できます。運用方針はMID360リポジトリのガイドに準拠しています（ODOMETRY経路推奨）。

- 参考: `LOITER.md`（運用ガイド）: `https://github.com/nonsaya/MID360/blob/main/LOITER.md`
- 参考: `MID360` リポジトリ（外部航法ブリッジ例）: `https://github.com/nonsaya/MID360`

## 前提
- Livox MID360 ドライバ、GLIM、MAVROS（ArduPilot接続済み）が起動済み
- GLIMが `/glim_ros/odom`（または `/glim_ros/odom_corrected`）を配信
- MAVROSがFCUと `connected: true`

## 1) パッケージ構成（本リポジトリに含済み）
- パッケージ: `ros2_ws/src/glim_mavros_bridge/glim_mavros_bridge`
- ノード:
  - `vision_bridge_node`: `/glim/pose` → `/mavros/vision_pose/pose`
  - `odom_bridge_node`（推奨）: `/glim_ros/odom(_corrected)` → `/mavros/odometry/out`（既定、切替可）
- Launch:
  - `vision_bridge.launch.py`
  - `odom_bridge.launch.py`

ビルド（すでにインストール済みの場合は不要）:
```bash
source /opt/ros/humble/setup.bash
cd /home/$USER/repo/loiter/ros2_ws
colcon build --packages-select glim_mavros_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 2) Vision 経路（代替）
GLIMの `/glim/pose` を MAVROS の `/mavros/vision_pose/pose` へ転送。
```bash
ros2 launch glim_mavros_bridge vision_bridge.launch.py \
  input_pose_topic:=/glim/pose \
  output_pose_topic:=/mavros/vision_pose/pose
```

## 3) ODOMETRY 経路（推奨）
GLIMの `Odometry` を MAVROSへ供給。多くの環境で安定します。
```bash
ros2 launch glim_mavros_bridge odom_bridge.launch.py \
  glim_namespace:=/glim_ros \
  use_corrected:=false \
  publish_rate_hz:=15.0 \
  odom_child_frame_id:=base_link \
  restamp_source:=now \
  reject_older_than_ms:=200.0 \
  publish_immediately:=true \
  target_topic:=/mavros/odometry/out   # 環境により /mavros/odometry/in を指定
```
- `glim_namespace`: `/glim_ros`（本環境の実トピック名）
- `use_corrected`: ループ閉じ後の `odom_corrected` を使う場合は `true`
- `target_topic`: ROS2のMAVROSでは `out` 側で取り込みが進む環境があるため切替可能
- タイムスタンプ: `restamp_source:=now` で鮮度問題を回避

## 4) 動作確認
- GLIM入力確認:
```bash
ros2 topic hz /glim_ros/odom
```
- ブリッジ出力確認:
```bash
ros2 topic hz /mavros/odometry/out
ros2 topic info /mavros/odometry/out
```
- EKF出力確認（ローカル位置）:
```bash
ros2 topic hz /mavros/local_position/pose
```
- MAVROS状態:
```bash
ros2 topic echo --once /mavros/state
```

## 5) よくあるハマりどころ
- トピック名の不一致: GLIMは `/glim` ではなく `/glim_ros` 名前空間で出る構成が多い
- タイムスタンプ鮮度: `restamp_source:=now` を使用
- frame整合性: `frame_id=odom` / `child_frame_id=base_link` を継続
- GLIMのPublisher: ビューワ拡張（例: `libstandard_viewer.so`）を無効化するとトピックが出ない構成あり（`LOITER.md`参照）
- MAVROSプラグイン購読先: 環境により `/mavros/odometry/in` ではなく `/mavros/odometry/out` で取り込みが進むケースあり（本ブリッジは `target_topic` で切替可）

## 6) 参考
- 運用ガイド: `https://github.com/nonsaya/MID360/blob/main/LOITER.md`
- MID360リポジトリ: `https://github.com/nonsaya/MID360`
