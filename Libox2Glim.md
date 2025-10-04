# Livox MID360 → GLIM セットアップ完全手順（Jetson/ROS2 Humble, CPU版）

目的: Jetson に Livox MID360 と GLIM（CPU 版）を新規構築し、/glim/odom, /glim/path, /glim/map, /tf を配信するまでを、このドキュメントだけで完了できるようにまとめます。

参考: GLIM 公式インストール手順（PPA/Source）: https://koide3.github.io/glim/installation.html

---

## 0. 前提・動作確認環境
- ハード: NVIDIA Jetson (JetPack 6.x, Ubuntu 22.04)
- ROS2: Humble
- LiDAR: Livox MID360（Ethernet）
- 本手順は GLIM CPU 版（PPA）を使用。Publisher は `librviz_viewer.so` が内包
- ネットワーク: MID360 と Jetson を同一 L2（直結/スイッチ）で接続

推奨: Jetson と PC の時刻同期（chrony）
```bash
sudo apt update && sudo apt install -y chrony
# NTPサーバがある環境であれば /etc/chrony/chrony.conf を適宜設定後: sudo systemctl restart chrony
# 単発合わせ（必要時）: sudo chronyc makestep
```

---

## 1. ROS2 Humble の導入（未導入の場合）
JetPack 6.x / Ubuntu 22.04 での一般的手順です。
```bash
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# 毎回の便利設定
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source /opt/ros/humble/setup.bash
```

---

## 2. Livox MID360 ドライバの導入
Livox 公式の ROS2 ドライバ（livox_ros_driver2）をソースからビルドします。

### 2.1 ワークスペース作成
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Livox ROS2ドライバ（公式）
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# （任意）MID360向けの簡易ブリンアップを使う場合は、手元の bringup パッケージを追加
# 例: livox_mid360_bringup（launch と設定の雛形がある場合）
# git clone <your_bringup_repo> livox_mid360_bringup
```

### 2.2 依存関係とビルド
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# 以後の端末で
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 2.3 ネットワーク設定（例）
MID360 は出荷時に固定 IP（例: 192.168.1.10）/UDP を使用します。Jetson の NIC を同一セグメントに設定してください。
```bash
# 例: Jetson 側 NIC (eth0) を 192.168.1.100/24 に設定
# GUI/NetworkManager でも OK。ここでは一時的な例のみ記載。
sudo ip addr add 192.168.1.100/24 dev eth0
sudo ip link set eth0 up
```

必要に応じて Livox のネット設定 JSON（mid360_net.json）を使い、ドライバから IP を設定できます（bringup に付属している場合）。

### 2.4 ドライバ起動
```bash
# 例1: 公式 driver のサンプル launch を使う場合
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360.launch.py   # ドライバによりファイル名は異なることあり

# 例2: bringup パッケージがある場合
# ros2 launch livox_mid360_bringup livox_mid360.launch.py
```

動作確認:
```bash
ros2 topic list | grep livox
ros2 topic hz /livox/imu           # 200Hz 前後
ros2 topic echo /livox/lidar --qos-profile sensor_data --qos-reliability best_effort -n 1
```

---

## 3. GLIM（CPU 版, PPA）インストール
GLIM 公式 PPA を使用します。Publisher は `librviz_viewer.so` で出力されます。
```bash
# PPA セットアップ
sudo apt install -y curl gpg
curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

# 依存
sudo apt update
sudo apt install -y libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev

# GTSAM (CPU) + GLIM (CPU)
sudo apt install -y libgtsam-points-dev
sudo apt install -y ros-humble-glim ros-humble-glim-ros

# 提供ファイルの例（確認）
dpkg -L ros-humble-glim | grep '\\.so$' || true
```

注意: 以前に /usr/local/lib に手動インストールした GTSAM があると ABI 衝突することがあります。不要であれば退避してください。
```bash
sudo mkdir -p /usr/local/lib/_bak_gtsam || true
sudo sh -c 'ls /usr/local/lib/libgtsam* 2>/dev/null | xargs -r -I{} mv "{}" /usr/local/lib/_bak_gtsam/'
sudo ldconfig
```

---

## 4. GLIM 設定
GLIM 設定ディレクトリ（例: `/home/nonsaya-n/glim_config`）を用意し、以下の JSON を配置します。パスは環境に合わせて読み替えてください。

### 4.1 config_ros.json（例）
```json
{
  "imu_topics": ["/livox/imu"],
  "points_topics": ["/livox/lidar"],
  "acc_scale": 9.80665,
  "base_frame_id": "",
  "enable_local_mapping": true,
  "enable_global_mapping": false,
  "points_time_offset": 0.0,
  "extension_modules": [
    "libmemory_monitor.so",
    "libstandard_viewer.so",
    "librviz_viewer.so",
    "libimu_validator.so"
  ]
}
```

### 4.2 config_sensors.json（Livox 時間設定の明示）
```json
{
  "autoconf_perpoint_times": false,
  "perpoint_relative_time": false,
  "perpoint_time_scale": 1e-9,
  "T_lidar_imu": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
}
```

### 4.3 config_odometry_cpu.json（代表例）
```json
{
  "target_downsampling_rate": 0.1,
  "max_iterations": 5,
  "initialization_mode": "LOOSE",
  "init_pose_damping_scale": 1e10,
  "initialization_window_size": 8.0,
  "use_isam2_dogleg": true,
  "isam2_relinearize_thresh": 0.05
}
```

必要に応じて `config_viewer.json` / `config_sub_mapping.json` / `config_global_mapping.json` も調整してください。

---

## 5. GLIM 起動と検証
Livox ドライバを起動済みにしてから GLIM を起動します。
```bash
source /opt/ros/humble/setup.bash
/opt/ros/humble/lib/glim_ros/glim_rosnode --ros-args -p config_path:=/home/nonsaya-n/glim_config
```
別端末で確認:
```bash
source /opt/ros/humble/setup.bash
ros2 node info /glim_ros
ros2 topic list | egrep '^/glim/(odom|path|map)$' || true
ros2 topic hz /glim/odom
```
起動直後は初期化中に姿勢が不安定なことがあります。10 秒程度は静止させて安定化させてください。

---

## 6. よくあるハマりどころと対処
- GTSAM ABI 衝突
  - /usr/local/lib に手動インストールした GTSAM があると PPA の GLIM と混在して `undefined symbol` が発生
  - 退避して `sudo ldconfig` 実行
- CUDA ランタイム不足（CUDA 版 GLIM を使う場合）
  - `libcudart.so.12` が無い等。JetPack 版に合わせて `cuda-cudart-12-6` などを導入後 `sudo ldconfig`
  - 本書は CPU 版を前提
- ROS ログディレクトリ
  - `env -i` のように `HOME` が無いと `Failed to get logging directory`。通常シェルで実行
- Livox のタイムスタンプ
  - `perpoint_*` を明示設定（上記参照）。`points_time_offset` は通常 0.0
- Publisher モジュール
  - `librviz_viewer.so` に Publisher 機能が内包され、`/glim/*` が配信されます
  - `libros_publisher.so` は不要
- DDS/通信
  - Docker 越し通信は DDS 設定で詰まることあり。今回はホスト上での実行を前提
- 時刻同期
  - IMU の高周波（200Hz）運用では時刻ずれの影響が出やすい。chrony 推奨

---

## 7. 片付け/運用メモ
- 起動順序: Livox ドライバ → GLIM
- 並走確認: `/livox/imu` `/livox/lidar` が安定配信 → `/glim/*` の生成
- 保存: 必要に応じて `ros2 bag record /glim/odom /glim/path /glim/map /tf`

---

## 8. 参考リンク
- GLIM 公式: https://koide3.github.io/glim/installation.html
- Livox ROS2 Driver: https://github.com/Livox-SDK/livox_ros_driver2
