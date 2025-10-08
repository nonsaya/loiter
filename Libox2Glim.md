## Jetson Orin Nano (8GB, JetPack 6.2) で Livox MID360 → GLIM を“ゼロから”構築する手順（ROS 2 Humble, CPU 版）

このドキュメント単体で、SSD をまっさらな状態から復旧・再構築できます。障害時は以下を上から順に実行してください。

参考: リポジトリ（Web 公開）: https://github.com/nonsaya/loiter

---

## 0. 前提/確認
- デバイス: Jetson Orin Nano 8GB（JetPack 6.2, Ubuntu 22.04）
- LiDAR: Livox MID360（Ethernet）
- IP 例: Jetson=192.168.1.50, MID360=192.168.1.3
- 目標: `/livox/lidar` に PointCloud2、`/livox/imu` に IMU。必要に応じて GLIM で `/glim/*` を出力。

推奨（時刻同期）:
```bash
sudo apt update && sudo apt install -y chrony
# 必要なら /etc/chrony/chrony.conf を調整後: sudo systemctl restart chrony
```

---

## 1. リポジトリ取得とブランチ作成
```bash
mkdir -p ~/repo && cd ~/repo
git clone https://github.com/nonsaya/loiter
cd loiter
git checkout -b setup/jetpack62-initial
```

---

## 2. ROS 2 Humble の導入
```bash
sudo apt update && sudo apt install -y locales curl gnupg2 lsb-release software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

---

## 3. ワークスペースと依存解決（rosdep）
```bash
mkdir -p /home/$USER/repo/loiter/ros2_ws/src
cd /home/$USER/repo/loiter/ros2_ws
sudo apt install -y python3-rosdep
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. Livox-SDK2 のインストール（/usr/local）
```bash
cd /home/$USER/repo/loiter
chmod +x scripts/*.sh
./scripts/build_livox_sdk2.sh
sudo ldconfig
```

補足: 依存が不足する場合は以下を追加
```bash
sudo apt install -y libpcl-dev ros-humble-pcl-conversions ros-humble-pcl-ros libapr1-dev libaprutil1-dev
```

---

## 5. ドライバ/ブリンアップの配置（本リポジトリに同梱）
- 本リポジトリの `ros2_ws/src` には以下を同梱:
  - `livox_ros_driver2`（公式）
  - `livox_mid360_bringup`（本リポジトリの起動設定）

ネットワーク設定の確認（すでに反映済み）:
```text
ros2_ws/src/livox_mid360_bringup/config/mid360_net.json
- host_net_info.*_ip  : 192.168.1.50
- lidar_configs[0].ip : 192.168.1.3

ros2_ws/src/livox_mid360_bringup/config/livox_mid360.yaml
- point_cloud_topic: "/livox/lidar"  # PointCloud2 を出力
```

---

## 6. ビルド（エラー時のワークアラウンド込み）
基本:
```bash
cd /home/$USER/repo/loiter/ros2_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select livox_ros_driver2
colcon build --symlink-install --packages-select livox_mid360_bringup
```

もし以下の CMake エラーが出る場合（環境や上流変更で発生することがあります）:
```
LIVOX_INTERFACES_INCLUDE_DIRECTORIES NOTFOUND
```
次のいずれかで解消できます。

- A) 本リポジトリ同梱の `livox_ros_driver2` に適用済みの修正を用いる（推奨）
  - すでに反映済みです。`rm -rf build install log` → 再ビルドしてください。

- B) 既存の clone に手動でワークアラウンドを適用
  - 目的: Humble で `rosidl_get_typesupport_target(...)` を用い、未定義変数参照を避ける
  - ポイント: `target_include_directories` から `${LIVOX_INTERFACES_INCLUDE_DIRECTORIES}` を一旦外し、取得できた場合のみ追加

---

## 7. 起動と確認（PointCloud2）
同一シェルで overlay を反映してから起動してください。
```bash
source /opt/ros/humble/setup.bash
source /home/$USER/repo/loiter/ros2_ws/install/setup.bash

ros2 launch livox_mid360_bringup livox_mid360.launch.py
```

別端末の確認例:
```bash
source /opt/ros/humble/setup.bash
source /home/$USER/repo/loiter/ros2_ws/install/setup.bash
ros2 topic list | grep livox
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --qos-profile sensor_data -n 1
```

期待トピック:
- `/livox/lidar` : sensor_msgs/PointCloud2（PointCloud2 固定: `xfer_format: 0`）
- `/livox/imu`   : sensor_msgs/Imu

---

## 8. GLIM（CPU, PPA）を使う場合（任意）
```bash
cd /home/$USER/repo/loiter
./scripts/setup_glim_cpu_ppa.sh
```
設定例（`/home/$USER/glim_config` に配置）:
```json
{
  "imu_topics": ["/livox/imu"],
  "points_topics": ["/livox/lidar"],
  "extension_modules": ["librviz_viewer.so"]
}
```
起動:
```bash
source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/glim_ros/glim_rosnode --ros-args -p config_path:=/home/$USER/repo/loiter/ros2_ws/src/glim_bringup/config/glim_config
```

### 8.1 推奨設定（Viewer/Downsampling）
- Viewer モジュール:
  - `librviz_viewer.so` は Publisher 機能を内包しており、これを有効にしないと `/glim/*` トピックが出ない構成があります（必須）。
  - ローカルの GLIM ビューアを起動させたくない場合は、`libstandard_viewer.so` をコメントアウト/除外してください。
  - 例（`ros2_ws/src/glim_bringup/config/glim_config/config_ros.json`）:
    ```json
    "extension_modules": [
      "libmemory_monitor.so",
      // "libstandard_viewer.so",  ← ビューアを抑止したい場合はコメントアウト
      "librviz_viewer.so"         ← ROS 出力に必須
    ]
    ```

- ダウンサンプリング（前処理）:
  - `random_downsample_target`: 5000 を推奨。
  - 例（`ros2_ws/src/glim_bringup/config/glim_config/config_preprocess.json`）:
    ```json
    {
      "preprocess": {
        "random_downsample_target": 5000
      }
    }
    ```


---

## 9. トラブルシュート
- `package 'xxx' not found`:
  - 同じシェル内で `source ros2_ws/install/setup.bash` を実行
- `liblivox_lidar_sdk_shared.so` not found:
  - `scripts/build_livox_sdk2.sh` を実行後 `sudo ldconfig`
- `LIVOX_INTERFACES_INCLUDE_DIRECTORIES NOTFOUND`:
  - 6章 B) のワークアラウンドを適用
- DDS で見えない:
  - 同一サブネット/ブロードキャスト許可、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

---

## 10. 運用メモ（復旧の最短手順）
```bash
git clone https://github.com/nonsaya/loiter ~/repo/loiter && cd ~/repo/loiter
git checkout -b setup/jetpack62-initial
sudo apt update && sudo apt install -y curl gnupg2 lsb-release python3-rosdep
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions libpcl-dev ros-humble-pcl-conversions ros-humble-pcl-ros libapr1-dev libaprutil1-dev
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && source /opt/ros/humble/setup.bash
sudo rosdep init || true && rosdep update
mkdir -p ~/repo/loiter/ros2_ws/src && cd ~/repo/loiter/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
cd ~/repo/loiter && chmod +x scripts/*.sh && ./scripts/build_livox_sdk2.sh && sudo ldconfig
cd ~/repo/loiter/ros2_ws && rm -rf build install log && source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select livox_ros_driver2
colcon build --symlink-install --packages-select livox_mid360_bringup
source ~/repo/loiter/ros2_ws/install/setup.bash
ros2 launch livox_mid360_bringup livox_mid360.launch.py
```

---

## 11. 参考リンク
- リポジトリ（公開）: https://github.com/nonsaya/loiter
- Livox ROS2 Driver2: https://github.com/Livox-SDK/livox_ros_driver2
- GLIM: https://koide3.github.io/glim/installation.html

---

## 12. MAVROS を“ゼロから”導入して ArduPilot（EKF3）に外部推定を供給する

目的: GLIM の自己位置（`/glim/odom`）を ArduPilot の EKF3 に外部推定として与え、LOITER でも安定ホバリングできる状態にする。

### 12.1 インストール（JetPack 6.2 / ROS 2 Humble）
```bash
sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavlink
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### 12.2 シリアル接続の前提設定（Jetson, `/dev/ttyTHS1:921600`）
```bash
# シリアル権限（初回のみ）
sudo usermod -aG dialout $USER
newgrp dialout

# getty の競合があれば停止（初回のみ）
sudo systemctl disable --now serial-getty@ttyTHS1.service || true

# ポート確認
ls -l /dev/ttyTHS1
```

### 12.3 MAVROS 起動（ROS 2 Humble の `apm.launch` は XML 版）
```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyTHS1:921600 tgt_system:=1
```

確認:
```bash
ros2 topic list | grep mavros
ros2 topic echo /mavros/state -n 1
```

### 12.4 EKF3 の外部推定設定（Mission Planner などで調整）
例（ExternalNav を使用）:
- EK3_SRC1_POSXY = 6
- EK3_SRC1_POSZ  = 6
- EK3_SRC1_VELXY = 6
- EK3_SRC1_VELZ  = 6
- EK3_SRC1_YAW   = 6
- （GPS 無し運用時）GPS_TYPE = 0

### 12.5 GLIM → MAVROS への Odom 供給
前提: GLIM から `/glim/odom`（`nav_msgs/Odometry`）が配信される。

- 送信先（MAVROS 側）: `/mavros/odometry/in`
- 一般的な frame 設定の例: `frame_id=map`（または `odom`）, `child_frame_id=base_link`
- 必要に応じてブリッジノードで frame 名や covariance を整えてリパブリッシュする。

最小確認フロー:
```bash
# ターミナル1: Livox → GLIM を起動
source /opt/ros/humble/setup.bash
ros2 launch livox_mid360_bringup livox_mid360.launch.py
# （別端末で GLIM 起動）
source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/glim_ros/glim_rosnode --ros-args -p config_path:=/home/$USER/glim_config

# ターミナル2: MAVROS をシリアルで接続
source /opt/ros/humble/setup.bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyTHS1:921600 tgt_system:=1

# ターミナル3: GLIM の Odom を確認
source /opt/ros/humble/setup.bash
ros2 topic echo /glim/odom -n 1

# （必要なら）/glim/odom → /mavros/odometry/in のブリッジを用意し、フレーム名等を整える
```

補足:
- MAVROS の launch は ROS 2 Humble では `apm.launch`（XML）。`.py` 版は同梱されないことがある。
- `install_geographiclib_datasets.sh` は地理座標の変換データを取得するために必須。
- `ttyTHS1` は Jetson の UART。ボーレート 921600 で FCU 側設定と一致させる。

---

## 13. GLIM → MAVROS ブリッジを“ゼロから”導入（/glim_ros/odom → /mavros/odometry/in）

目的: GLIM の `nav_msgs/Odometry` を MAVROS の `/mavros/odometry/in` へ供給（EKF3 に外部推定として採用させる）。

### 13.1 パッケージ構成（本リポジトリ同梱）
- 位置: `ros2_ws/src/glim_mavros_bridge`
- ノード: `glim_mavros_bridge/bridge_node.py`
- 起動: `glim_mavros_bridge/launch/bridge.launch.py`

### 13.2 ビルド
```bash
cd /home/$USER/repo/loiter/ros2_ws
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install --merge-install --packages-select glim_mavros_bridge
```

### 13.3 起動（最小例: ODOMETRY のみ）
```bash
source /opt/ros/humble/setup.bash
source /home/$USER/repo/loiter/ros2_ws/install/setup.bash
ros2 launch glim_mavros_bridge bridge.launch.py \
  mirror_to_odometry_out:=false publish_vision_topics:=false \
  override_stamp_with_now:=false frame_id:=odom child_frame_id:=base_link
```

### 13.4 ブリッジ パラメータ
- `input_topic`（既定: `/glim_ros/odom`）: GLIM のオドメトリ入力
- `output_topic`（既定: `/mavros/odometry/in`）: MAVROS への出力
- `frame_id`（既定: `odom` 推奨）: 基準座標系（`odom` or `map`）
- `child_frame_id`（既定: `base_link`）: 機体座標
- `override_stamp_with_now`（既定: false）: 時刻を現在時刻に上書き（通常は false で GLIM の時刻を使用）
- `pose_cov_diag`（既定: `[0.02, 0.02, 0.05, 0.01, 0.01, 0.02]`）: [x,y,z,roll,pitch,yaw] の対角分散
- `twist_cov_diag`（既定: `[0.05, 0.05, 0.10, 0.02, 0.02, 0.02]`）: [vx,vy,vz,wx,wy,wz] の対角分散
- `publish_vision_topics`（既定: false）: 互換の VISION 経路（`/mavros/vision_pose/pose_cov`, `/mavros/vision_speed/speed_twist`）も同時出力
- `mirror_to_odometry_out`（既定: false）: 監視用に `/mavros/odometry/out` へ“ミラー出力”（機能検証用。採用とは無関係）

補足:
- 速度は EKF の期待（機体座標）に合わせ、ブリッジ内でワールド→ボディへ変換。
- covariance が全ゼロ入力の場合は最小対角分散を自動付与（EKF に無視されにくくする）。

### 13.5 推奨プリセット（まずは中庸→微調整）
- 速応型（追従重視）
```bash
pose_cov_diag:="[0.02, 0.02, 0.05, 0.01, 0.01, 0.02]"
twist_cov_diag:="[0.05, 0.05, 0.10, 0.02, 0.02, 0.02]"
```
- 中庸（最初に推奨）
```bash
pose_cov_diag:="[0.03, 0.03, 0.08, 0.02, 0.02, 0.03]"
twist_cov_diag:="[0.08, 0.08, 0.15, 0.03, 0.03, 0.03]"
```
- 平滑型（揺れ抑制）
```bash
pose_cov_diag:="[0.05, 0.05, 0.12, 0.03, 0.03, 0.05]"
twist_cov_diag:="[0.12, 0.12, 0.20, 0.04, 0.04, 0.04]"
```

### 13.6 チューニングの指針（より詳しく）
- 平面の小刻み振動（XY）
  - 影響変数: `pose_cov_diag` の x,y、`twist_cov_diag` の vx,vy
  - 対応: それぞれ +0.01〜+0.03 ずつ増やす（例: x,y を 0.03→0.05、vx,vy を 0.08→0.10）
- 上下のふらつき（Z）
  - 影響変数: `pose_cov_diag` の z、`twist_cov_diag` の vz
  - 対応: z を 0.08→0.10〜0.12、vz を 0.15→0.18〜0.20
- ヨーの微振動（Yaw）
  - 影響変数: `pose_cov_diag` の yaw、`twist_cov_diag` の wz
  - 対応: yaw を 0.03→0.05、wz を 0.03→0.05 に増やす

調整の流れ:
1) 中庸プリセットで起動し、RViz2 で `/glim_ros/pose` と `/mavros/local_position/pose` を重ねて観察（30–60秒）
2) 揺れ方向に応じて該当軸の分散を +0.01〜+0.03 刻みで増やす
3) 応答が遅くなり過ぎたら 1段階だけ戻す

### 13.7 frame の選び方
- `frame_id=odom` 推奨（局所一貫でジャンプが少ない）
- 地図整合（再ローカライズ）を強く使う場合は `map` も可（ジャンプ許容）
- `child_frame_id=base_link`（機体座標）

### 13.8 タイムスタンプ
- 既定: `override_stamp_with_now=false`（GLIM の時刻を使用）
- 時刻ズレ疑い時のみ true を一時試験（通常は不要）

### 13.9 監視用の “ミラー出力”
- `/mavros/odometry/out` は FCU→MAVROS→ROS の出力側。採用可否の判断には使わない。
- 可視化や比較が必要な場合のみ `mirror_to_odometry_out:=true` で有効化（既定は false）。

### 13.10 トラブルシュート
- `/mavros/local_position/pose` が更新しない
  - `/mavros/odometry/in` が 20Hz で入っているか
  - ブリッジの `frame_id`, `covariance` を調整
  - FCU 再起動後、収束まで数十秒静止
- VISION 経路と併用したい
  - `publish_vision_topics:=true` で `/mavros/vision_pose/pose_cov` と `/mavros/vision_speed/speed_twist` を同時出力

