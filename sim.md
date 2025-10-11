# シミュレーション手順（WSL2 + ArduPilot SITL + MAVROS + RViz2/QGC）

Windows 上の WSL2（Ubuntu）で ArduPilot SITL を動かし、Jetson 側の ROS 2 + MAVROS と接続して、GUI（QGroundControl / RViz2）で確認するまでをゼロから説明します。

## 前提
- Windows 10/11（管理者 PowerShell が使える）
- Jetson に ROS 2（例: Humble）+ MAVROS が導入済み
- Windows（WSL2）と Jetson が同一ネットワーク

---

## 1) WSL2 + Ubuntu の導入（Windows）
1. 管理者 PowerShell を起動
2. そのまま導入（再起動→初期設定）
```powershell
wsl --install -d Ubuntu-22.04
```

---

## 2) ArduPilot SITL の準備（WSL2 Ubuntu）
```bash
# 基本ツール
sudo apt update
sudo apt install -y git python3-pip build-essential

# ArduPilot を取得（サブモジュール込み）
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# 依存インストール（公式）
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# SITL ビルド（Copter）
./waf configure --board sitl
./waf copter
```

---

## 3) SITL の起動（単体確認）
```bash
# 最小（コンソール/地図なし）
sim_vehicle.py -v ArduCopter
# 停止: Ctrl+C
```
（GUI を出す場合は WSL2 に X サーバが必要）
```bash
sim_vehicle.py -v ArduCopter --console --map
```

---

## 4) SITL を Jetson へ出力
Jetson の IP を例として `192.168.0.98` とします。
```bash
# WSL2（Ubuntu）で再起動
sim_vehicle.py -v ArduCopter --out 192.168.0.98:14550
```
Jetson 受信確認（任意）:
```bash
sudo tcpdump -i any udp port 14550 -nn -c 5
```

---

## 5) Jetson で MAVROS を起動
```bash
# 停止（起動中なら）
pkill -f mavros_node; pkill -f mavros_router

# 起動（/uas1/mavros/* 名前空間）
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://0.0.0.0:14550@ -p fcu_protocol:=v2.0

# 接続確認
ros2 topic echo /uas1/mavros/state --once   # connected: true でOK
```
名前空間を無くして `/mavros/*` にしたい場合:
```bash
pkill -f mavros_node; pkill -f mavros_router
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://0.0.0.0:14550@ -p fcu_protocol:=v2.0 -p uas_prefix:=""
```

---

## 6) 最小ミッション（手動シーケンス）
順に 1 行ずつ実行:
```bash
# GUIDED
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'GUIDED'}"
# アーム
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
# 離陸（1.0m）
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 1.0, min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0}"
# 10秒待機
sleep 10
# X（前）方向に 1.0 m 進む（位置 setpoint）
ros2 topic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 1.0}}}" -r 20
# 別端末で 6〜8 秒程度配信し続け、収束を待つ（Ctrl+Cで停止）
# 収束が見えたら配信を停止 → そのままホバリング
sleep 2
# LAND
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'LAND'}"
# 接地後にディスアーム
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```
（`/uas1/mavros/*` の場合は先頭を `/uas1/mavros/` に置き換え）

---

## 7) GUI で見る
- QGroundControl（Windows）
  - SITL を Jetson と Windows に同時出力
  ```bash
  sim_vehicle.py -v ArduCopter --out 192.168.0.98:14550 --out 127.0.0.1:14550
  ```
  - QGC を起動（UDP 14550 を自動受信）
- RViz2（Jetson）
  ```bash
  source /opt/ros/humble/setup.bash
  rviz2
  ```
  - Fixed Frame: `odom`
  - 位置: `/mavros/local_position/pose`（History Length を増やす）
  - 軌跡: `/mavros/local_position/odom` を Odometry 表示、または `nav_msgs/Path` ノードで線表示

---

## 8) よくあるハマり
- connected:false → SITL の `--out` 先、FW、`fcu_url` を再確認
- 空文字パラメータは `-p name:=""` と書く
- `UDP URL should contain @!` → `fcu_url` 末尾に `@`
- プロポ操作は SITL に届かない → モード切替は QGC または ROS サービス

---

## 9) 次のステップ
- `loiter_mission` ノードで setpoint 配信を確認
- SITL=開発PC、MAVROS+ノード=Jetson の分離運用
- 受入基準に沿った自動ミッション（TDD/MVP）へ拡張
