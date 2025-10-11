# ステージ1 最小ミッション手順（実機／SITL 共通）

目的: アーム → 離陸1m → 10秒ホバ → X+1m → LAND → ディスアーム

## 0. 安全準備
- プロペラ無しで動作確認 → OK後に装着
- 送信機: 開始時はスロットル最低（介入時 LOITER 切替なら中央）
- `MOT_SPIN_ARMED` は低め（必要なら 0）

## 1. 一発起動（LiDAR/GLIM/MAVROS/ブリッジ）
```bash
cd /home/nonsaya-n/repo/loiter
scripts/launch_loiter_tmux.sh
```

## 2. 接続と外部オドメ確認（Jetson）
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/state --once                # connected:true
ros2 topic hz   /mavros/odometry/in                 # ≈20Hz
ros2 topic echo /mavros/local_position/pose --once  # frame_id = map/odom を把握
```

## 3. 室内・GPSなしでの Global Origin 設定
- FCU起動直後（DISARMのまま）に 5Hz で 3–5 秒 Publish
```bash
ros2 topic pub -r 5 /mavros/global_position/set_gp_origin geographic_msgs/msg/GeoPointStamped \
'{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, position: {latitude: 35.0, longitude: 135.0, altitude: 0.0}}'
```
- 確認
```bash
ros2 topic echo /mavros/statustext/recv --once
ros2 topic echo /mavros/global_position/gp_origin --once
```

参考: 無GPS＋外部オドメ運用の代表パラメータ（設定後はFCU再起動）
```bash
# 無GPS
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'GPS_TYPE', value: {integer: 0}}"
# EKF3 + 外部Nav
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'AHRS_EKF_TYPE', value: {integer: 3}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_POSXY', value: {integer: 6}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_VELXY', value: {integer: 6}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_YAW',  value: {integer: 6}}"
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_POSZ', value: {integer: 1}}"  # Baro
ros2 service call /mavros/param/set mavros_msgs/srv/ParamSet "{param_id: 'EK3_SRC1_VELZ', value: {integer: 6}}"
```

## 4. 最小ミッション（手動ステップ）
```bash
# GUIDED
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
# ARM
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
# 離陸 1.0m
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 1.0, min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0}"
# 10秒待機
sleep 10
# X+1.0m（別端末で20Hz, 6–8秒流して Ctrl+C）
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped \
"{header:{frame_id:'map'}, pose:{position:{x:1.0, y:0.0, z:1.0}}}" -r 20
# LAND → 接地後 DISARM
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'LAND'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

## 5. 介入・退避
```bash
# 高度維持で退避
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'LOITER'}"
# 完全マニュアル（要注意）
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'STABILIZE'}"
```
- 退避時は setpoint 配信を必ず停止（配信端末で Ctrl+C）。

## 6. トラブル時の基本確認
```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/statustext/recv --once
ros2 topic echo /mavros/estimator_status --once
ros2 topic hz   /mavros/odometry/in
```
- “PreArm: AHRS: waiting for home” → 再起動直後に Global Origin を Publish
- 離陸が通らなければ Z=目標setpoint を20Hzで配信して反応確認

## 付録: PowerShell から WSL で SITL 一発起動
```powershell
wsl -d Ubuntu-22.04 -- bash -lc "cd ~/ardupilot && sim_vehicle.py -v ArduCopter --out 192.168.0.98:14550"
```
