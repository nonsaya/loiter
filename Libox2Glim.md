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
/opt/ros/humble/lib/glim_ros/glim_rosnode --ros-args -p config_path:=/home/$USER/glim_config
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
